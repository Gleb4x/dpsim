// SPDX-License-Identifier: Apache-2.0

#include <dpsim/Interface.h>
#include <dpsim/InterfaceWorker.h>

using namespace CPS;

namespace DPsim {

    void Interface::open() {
        mInterfaceWorker->open();
        mOpened = true;

        if (!mImportAttrsDpsim.empty()) {
            mInterfaceReaderThread = std::thread(Interface::ReaderThread(mQueueInterfaceToDpsim, mInterfaceWorker, mOpened));
        }
        if (!mExportAttrsDpsim.empty()) {
            mInterfaceWriterThread = std::thread(Interface::WriterThread(mQueueDpsimToInterface, mInterfaceWorker));
        }
    }

    void Interface::close() {
	    mOpened = false;
        mQueueDpsimToInterface->emplace(AttributePacket {
            nullptr,
            0,
            0,
            AttributePacketFlags::PACKET_CLOSE_INTERFACE
        });

        if (!mExportAttrsDpsim.empty()) {
            mInterfaceWriterThread.join();
        }

        if (!mImportAttrsDpsim.empty()) {
            mInterfaceReaderThread.join();
        }
        mInterfaceWorker->close();
    }

    CPS::Task::List Interface::getTasks() {
        auto tasks = CPS::Task::List();
        if (!mImportAttrsDpsim.empty()) {
            tasks.push_back(std::make_shared<Interface::PreStep>(*this));
        }
        if (!mExportAttrsDpsim.empty()) {
            tasks.push_back(std::make_shared<Interface::PostStep>(*this));
        }
        return tasks;
    }

    void Interface::PreStep::execute(Real time, Int timeStepCount) {
        if (timeStepCount % mIntf.mDownsampling == 0)
            mIntf.popDpsimAttrsFromQueue();
    }

    void Interface::PostStep::execute(Real time, Int timeStepCount) {
        if (timeStepCount % mIntf.mDownsampling == 0)
            mIntf.pushDpsimAttrsToQueue();
    }

    void Interface::addImport(CPS::AttributeBase::Ptr attr, bool blockOnRead) {
        if (mOpened) {
            mLog->error("Cannot modify interface configuration after simulation start!");
            std::exit(1);
        }

        mImportAttrsDpsim.emplace_back(std::make_tuple(attr, 0, blockOnRead));
    }

    void Interface::addExport(CPS::AttributeBase::Ptr attr) {
        if (mOpened) {
            mLog->error("Cannot modify interface configuration after simulation start!");
            std::exit(1);
        }

        mExportAttrsDpsim.emplace_back(std::make_tuple(attr, 0));
    }

    void Interface::setLogger(CPS::Logger::Log log) {
        mLog = log;
        if (mInterfaceWorker != nullptr)
        {
            mInterfaceWorker->mLog = log;
        }	
	}

    void Interface::popDpsimAttrsFromQueue() {
        AttributePacket receivedPacket = {
            nullptr,
            0,
            0,
            AttributePacketFlags::PACKET_NO_FLAGS
        };
        UInt currentSequenceId = mNextSequenceInterfaceToDpsim;

        //Wait for and dequeue all attributes that read should block on
        //The std::find_if will look for all attributes that have not been updated in the current while loop (i. e. whose sequence ID is lower than the next expected sequence ID)
        while (std::find_if(
                mImportAttrsDpsim.cbegin(),
                mImportAttrsDpsim.cend(),
                [currentSequenceId](auto attrTuple) {
                    return std::get<2>(attrTuple) && std::get<1>(attrTuple) < currentSequenceId;
                }) != mImportAttrsDpsim.cend()) {
            mQueueInterfaceToDpsim->wait_dequeue(receivedPacket);
            if (!std::get<0>(mImportAttrsDpsim[receivedPacket.attributeId])->copyValue(receivedPacket.value)) {
                mLog->warn("Failed to copy received value onto attribute in Interface!");
            }
            std::get<1>(mImportAttrsDpsim[receivedPacket.attributeId]) = receivedPacket.sequenceId;
            mNextSequenceInterfaceToDpsim = receivedPacket.sequenceId + 1;
        }

        //Fetch all remaining queue packets
        while (mQueueInterfaceToDpsim->try_dequeue(receivedPacket)) {
            if (!std::get<0>(mImportAttrsDpsim[receivedPacket.attributeId])->copyValue(receivedPacket.value)) {
                mLog->warn("Failed to copy received value onto attribute in Interface!");
            }
            std::get<1>(mImportAttrsDpsim[receivedPacket.attributeId]) = receivedPacket.sequenceId;
            mNextSequenceInterfaceToDpsim = receivedPacket.sequenceId + 1;
        }
    }

    void Interface::pushDpsimAttrsToQueue() {
        for (UInt i = 0; i < mExportAttrsDpsim.size(); i++) {
            mQueueDpsimToInterface->emplace(AttributePacket {
                std::get<0>(mExportAttrsDpsim[i])->cloneValueOntoNewAttribute(),
                i,
                std::get<1>(mExportAttrsDpsim[i]),
                AttributePacketFlags::PACKET_NO_FLAGS
            });
            std::get<1>(mExportAttrsDpsim[i]) = mCurrentSequenceDpsimToInterface;
            mCurrentSequenceDpsimToInterface++;
        }
    }

    void Interface::WriterThread::operator() () const {
        bool interfaceClosed = false;
        std::vector<Interface::AttributePacket> attrsToWrite;
        while (!interfaceClosed) {
            AttributePacket nextPacket = {
                nullptr,
                0,
                0,
                AttributePacketFlags::PACKET_NO_FLAGS
            };

            //Wait for at least one packet
            mQueueDpsimToInterface->wait_dequeue(nextPacket);
            if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
                interfaceClosed = true;
            } else {
                attrsToWrite.push_back(nextPacket);
            }

            //See if there are more packets
            while(mQueueDpsimToInterface->try_dequeue(nextPacket)) {
                if (nextPacket.flags & AttributePacketFlags::PACKET_CLOSE_INTERFACE) {
                    interfaceClosed = true;
                } else {
                    attrsToWrite.push_back(nextPacket);
                }
            }
            mInterfaceWorker->writeValuesToEnv(attrsToWrite);
        }
    }

    void Interface::ReaderThread::operator() () const {
        std::vector<Interface::AttributePacket>  attrsRead;
        while (mOpened) { //TODO: As long as reading blocks, there is no real way to force-stop thread execution from the dpsim side
            mInterfaceWorker->readValuesFromEnv(attrsRead);
            for (auto packet : attrsRead) {
                mQueueInterfaceToDpsim->enqueue(packet);
            }
            attrsRead.clear();
        }
    }

}

