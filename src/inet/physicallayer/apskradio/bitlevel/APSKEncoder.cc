//
// Copyright (C) 2014 OpenSim Ltd.
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//

#include <algorithm>
#include "inet/common/packet/chunk/BitsChunk.h"
#include "inet/physicallayer/apskradio/bitlevel/APSKEncoder.h"
#include "inet/physicallayer/apskradio/packetlevel/APSKPhyHeader_m.h"

namespace inet {

namespace physicallayer {

Define_Module(APSKEncoder);

APSKEncoder::APSKEncoder() :
    code(nullptr),
    scrambler(nullptr),
    fecEncoder(nullptr),
    interleaver(nullptr)
{
}

APSKEncoder::~APSKEncoder()
{
    delete code;
}

void APSKEncoder::initialize(int stage)
{
    if (stage == INITSTAGE_LOCAL) {
        scrambler = dynamic_cast<IScrambler *>(getSubmodule("scrambler"));
        fecEncoder = dynamic_cast<IFECCoder *>(getSubmodule("fecEncoder"));
        interleaver = dynamic_cast<IInterleaver *>(getSubmodule("interleaver"));
    }
    else if (stage == INITSTAGE_PHYSICAL_LAYER) {
        const IScrambling *scrambling = scrambler != nullptr ? scrambler->getScrambling() : nullptr;
        const ConvolutionalCode *forwardErrorCorrection = fecEncoder != nullptr ? check_and_cast<const ConvolutionalCode *>(fecEncoder->getForwardErrorCorrection()) : nullptr;
        const IInterleaving *interleaving = interleaver != nullptr ? interleaver->getInterleaving() : nullptr;
        code = new APSKCode(forwardErrorCorrection, interleaving, scrambling);
    }
}

std::ostream& APSKEncoder::printToStream(std::ostream& stream, int level) const
{
    stream << "APSKEncoder";
    if (level <= PRINT_LEVEL_DETAIL)
        stream << ", code = " << printObjectToString(code, level + 1);
    if (level <= PRINT_LEVEL_TRACE)
        stream << ", scrambler = " << printObjectToString(scrambler, level + 1)
               << ", fecEncoder = " << printObjectToString(fecEncoder, level + 1)
               << ", interleaver = " << printObjectToString(interleaver, level + 1);
    return stream;
}

const ITransmissionBitModel *APSKEncoder::encode(const ITransmissionPacketModel *packetModel) const
{
    int bitLength = 1000;
    int repeatCount = 10000;
//    for (auto grossBER : {0.1, 0.2, 0.4, 0.8}) {
//    for (auto grossBER : {0.01, 0.02, 0.04, 0.1}) {
    for (auto grossBER : {0.001, 0.002, 0.004, 0.01, 0.02, 0.04, 0.1}) {
        int bitLevelErrorCount = 0;
        for (int r = 0; r < repeatCount; r++) {
            BitVector *data = new BitVector();
            for (int i = 0; i < bitLength; i++)
                data->appendBit(uniform(0, 1) < 0.5);
            auto encodedData = fecEncoder->encode(*data);
            int bitErrorCount = encodedData.getSize() * grossBER;
            std::vector<int> bitErrorIndices;
            for (int i = 0; i < bitErrorCount; i++) {
                while (true) {
                    int index = intuniform(0, encodedData.getSize() - 1);
                    if (std::find(bitErrorIndices.begin(), bitErrorIndices.end(), index) == bitErrorIndices.end()) {
                        bitErrorIndices.push_back(index);
                        break;
                    }
                }
            }
            for (auto bitErrorIndex : bitErrorIndices)
                encodedData.toggleBit(bitErrorIndex);
//            int bitErrorCount = 0;
//            for (int i = 0; i < (int)encodedData.getSize(); i++) {
//                if (uniform(0, 1) < grossBER) {
//                    bitErrorCount++;
//                    encodedData.toggleBit(i);
//                }
//            }
//            double measuredBER = (double)bitErrorCount / encodedData.getSize();
//            std::cout << "Parameter gross BER: " << grossBER << ", measured gross BER: " << measuredBER << std::endl;
            auto fecDecoder = check_and_cast<IFECCoder *>(getModuleByPath("^.^.receiver.decoder.fecDecoder"));
            auto decodedDataAndSuccess = fecDecoder->decode(encodedData);
            auto decodedData = decodedDataAndSuccess.first;
            auto hasBitError = !decodedDataAndSuccess.second;
            for (int i = 0; i < bitLength; i++)
                if (data->getBit(i) != decodedData.getBit(i))
                    hasBitError = true;
            if (hasBitError)
                bitLevelErrorCount++;
        }
        double bitLevelPER = (double)bitLevelErrorCount / repeatCount;
        double netBER = fecEncoder->getForwardErrorCorrection()->computeNetBitErrorRate(grossBER);
        double packetLevelPER = 1 - pow(1 - netBER, bitLength);
        std::cout << "Bit length: " << bitLength << ", gross BER: " << grossBER << ", net BER: " << netBER << ", packet level PER: " << packetLevelPER << ", bit level PER: " << bitLevelPER << std::endl;
    }


    auto packet = packetModel->getPacket();
    const auto& apskPhyHeader = packet->peekHeader<APSKPhyHeader>();
    auto length = packet->getTotalLength();
    BitVector *encodedBits;
    if (b(length).get() % 8 == 0)
        encodedBits = new BitVector(packet->peekAllBytes()->getBytes());
    else {
        encodedBits = new BitVector();
        const auto& bitsChunk = packet->peekAllBits();
        for (int i = 0; i < b(length).get(); i++)
            encodedBits->appendBit(bitsChunk->getBit(i));
    }
    const IScrambling *scrambling = nullptr;
    if (scrambler) {
        *encodedBits = scrambler->scramble(*encodedBits);
        scrambling = scrambler->getScrambling();
        EV_DEBUG << "Scrambled bits are: " << *encodedBits << endl;
    }
    const IForwardErrorCorrection *forwardErrorCorrection = nullptr;
    if (fecEncoder) {
        *encodedBits = fecEncoder->encode(*encodedBits);
        forwardErrorCorrection = fecEncoder->getForwardErrorCorrection();
        EV_DEBUG << "FEC encoded bits are: " << *encodedBits << endl;
    }
    const IInterleaving *interleaving = nullptr;
    if (interleaver) {
        *encodedBits = interleaver->interleave(*encodedBits);
        interleaving = interleaver->getInterleaving();
        EV_DEBUG << "Interleaved bits are: " << *encodedBits << endl;
    }
    b netHeaderLength = apskPhyHeader->getChunkLength();
    if (forwardErrorCorrection == nullptr)
        return new TransmissionBitModel(netHeaderLength, packetModel->getBitrate(), length - netHeaderLength, packetModel->getBitrate(), encodedBits, forwardErrorCorrection, scrambling, interleaving);
    else {
        b grossHeaderLength = b(forwardErrorCorrection->getEncodedLength(b(netHeaderLength).get()));
        bps grossBitrate = packetModel->getBitrate() / forwardErrorCorrection->getCodeRate();
        return new TransmissionBitModel(grossHeaderLength, grossBitrate, length - grossHeaderLength, grossBitrate, encodedBits, forwardErrorCorrection, scrambling, interleaving);
    }
}

} // namespace physicallayer

} // namespace inet

