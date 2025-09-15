#include "configuration.h"

#if !MESHTASTIC_EXCLUDE_BLUETOOTH && defined(CONFIG_BT_NIMBLE_EXT_ADV) && RADIOLIB_EXCLUDE_BLE_MESH_RF != 1 && defined(NIMBLE_TWO)

#include "NodeDB.h"
#include "PowerMon.h"
#include "mesh/Router.h"

#include "BLEMeshInterface.h"

#ifdef __cplusplus
extern "C" {
#endif
// access internal ESP32 functions
extern int rom_phy_get_noisefloor(void);
// extern int rom_read_hw_noisefloor(void);
#ifdef __cplusplus
}
#endif

float BLEMeshInterface::getNoiseFloor()
{
    float dbm = rom_phy_get_noisefloor() / 4;
    LOG_DEBUG("BLE Noise floor: %f", dbm);
    return dbm;
}

// BLE RX event
void BLEMeshInterface::onResult(const NimBLEAdvertisedDevice *advertisedDevice)
{
    std::string raw = advertisedDevice->getServiceData(NimBLEUUID(MESH_RF_UUID));
    int length = raw.size();
    if (!length) {
        return;
    }

    NimBLEAddress addr = advertisedDevice->getAddress();
    int rssi = advertisedDevice->getRSSI();

    LOG_INFO("Advertised RSSI: %d Address: %s", rssi, addr.toString().c_str());
    LOG_INFO("raw (%d): %.*s", length, length, (char *)raw.data());
    if (length > sizeof(rxRadioBuffer)) {
        LOG_ERROR("Incoming Data to Big.");
        return;
    }

    size_t copyLen = length < sizeof(rxRadioBuffer) ? length : sizeof(rxRadioBuffer);
    memcpy(&rxRadioBuffer, raw.data(), copyLen);

    int32_t payloadLen = copyLen - sizeof(PacketHeader);

    meshtastic_MeshPacket *mp = packetPool.allocZeroed();

    // Keep the assigned fields in sync with src/mqtt/MQTT.cpp:onReceiveProto
    mp->from = rxRadioBuffer.header.from;
    mp->to = rxRadioBuffer.header.to;
    mp->id = rxRadioBuffer.header.id;
    mp->channel = rxRadioBuffer.header.channel;
    assert(HOP_MAX <= PACKET_FLAGS_HOP_LIMIT_MASK); // If hopmax changes, carefully check this code
    mp->hop_limit = rxRadioBuffer.header.flags & PACKET_FLAGS_HOP_LIMIT_MASK;
    mp->hop_start = (rxRadioBuffer.header.flags & PACKET_FLAGS_HOP_START_MASK) >> PACKET_FLAGS_HOP_START_SHIFT;
    mp->want_ack = !!(rxRadioBuffer.header.flags & PACKET_FLAGS_WANT_ACK_MASK);
    mp->via_mqtt = !!(rxRadioBuffer.header.flags & PACKET_FLAGS_VIA_MQTT_MASK);
    // If hop_start is not set, next_hop and relay_node are invalid (firmware <2.3)
    mp->next_hop = mp->hop_start == 0 ? NO_NEXT_HOP_PREFERENCE : rxRadioBuffer.header.next_hop;
    mp->relay_node = mp->hop_start == 0 ? NO_RELAY_NODE : rxRadioBuffer.header.relay_node;

    mp->rx_rssi = lround(rssi);
    mp->rx_snr = mp->rx_rssi - getNoiseFloor();

    mp->which_payload_variant = meshtastic_MeshPacket_encrypted_tag; // Mark that the payload is still encrypted at this point
    assert(((uint32_t)payloadLen) <= sizeof(mp->encrypted.bytes));

    memcpy(mp->encrypted.bytes, rxRadioBuffer.payload, payloadLen);
    mp->encrypted.size = payloadLen;

    mp->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT1;

    printPacket("BLE Lora RX", mp);

    if (router) {
        mp->transport_mechanism = meshtastic_MeshPacket_TransportMechanism_TRANSPORT_LORA_ALT1;
        router->enqueueReceivedMessage(mp);
    }
}

// BLE RX stopped
void BLEMeshInterface::onScanEnd(const NimBLEScanResults &results, int reason)
{
    LOG_ERROR("BLE Scan ended reason: %d, restarting", reason);
    powerMon->clearState(meshtastic_PowerMon_State_Lora_RXOn);
    notifyLater(100, BLE_SCAN_STOPPED, false);
}

// BLE TX finished
void BLEMeshInterface::onStopped(NimBLEExtAdvertising *pAdv, int reason, uint8_t instId)
{
    if (instId == 0) {
        LOG_WARN("BLE Advertising restart");
        NimBLEExtAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
        pAdvertising->start(0, 0, 0);
        return;
    }
    if (instId != BLE_MESH_ADV_INST_ID) {
        return;
    }
    LOG_INFO("BLE TX finished");

    auto txp = sendingPacket;
    sendingPacket = NULL;
    if (txp) {
        printPacket("Completed sending", txp);
        packetPool.release(txp);
    }

    powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn);
    notifyLater(1, TRANSMIT_DELAY_COMPLETED, false);
}

BLEMeshInterface::BLEMeshInterface(NimbleBluetooth *nimbleBluetooth) : NotifiedWorkerThread("BLEmesh")
{
    LOG_DEBUG("BLEMeshInterface()");
    this->nimbleBluetooth = nimbleBluetooth;
}

/// Initialise the Driver transport hardware and software.
/// Make sure the Driver is properly configured before calling init().
/// \return true if initialisation succeeded.
bool BLEMeshInterface::init()
{
    LOG_WARN("BLEMeshInterface init");
    if (!nimbleBluetooth->isActive()) {
        nimbleBluetooth->setup();
    }

    NimBLEExtAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    pAdvertising->setCallbacks(this, false);

    NimBLEScan *pBLEScan = NimBLEDevice::getScan();
    /*
    if (pBLEScan->isScanning()) {
        return true;
    }
    */

    LOG_INFO("BLEMeshInterface Start New BLE Scan...");
    pBLEScan->setScanCallbacks(this, true);
    pBLEScan->setActiveScan(false);
    pBLEScan->setMaxResults(0);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(100);

    powerMon->setState(meshtastic_PowerMon_State_Lora_RXOn);
    if (!pBLEScan->start(0, false, true)) {
        LOG_ERROR("BLEMeshInterface BLE Scan start Failed!");
        return false;
    }
    return true;
}

bool BLEMeshInterface::reconfigure()
{
    LOG_WARN("BLEMeshInterface reconfigure");
    return RADIOLIB_ERR_NONE;
};

// can we Sleep and BLE scan at the same time?
bool BLEMeshInterface::canSleep()
{
    return false;
}

meshtastic_QueueStatus BLEMeshInterface::getQueueStatus()
{
    meshtastic_QueueStatus qs;

    qs.res = qs.mesh_packet_id = 0;
    qs.free = txQueue.getFree();
    qs.maxlen = txQueue.getMaxLen();

    return qs;
}

/** Attempt to cancel a previously sent packet.  Returns true if a packet was found we could cancel */
bool BLEMeshInterface::cancelSending(NodeNum from, PacketId id)
{
    auto p = txQueue.remove(from, id);
    if (p)
        packetPool.release(p); // free the packet we just removed

    bool result = (p != NULL);
    LOG_DEBUG("cancelSending id=0x%x, removed=%d", id, result);
    return result;
}

/** Attempt to find a packet in the TxQueue. Returns true if the packet was found. */
bool BLEMeshInterface::findInTxQueue(NodeNum from, PacketId id)
{
    return txQueue.find(from, id);
}

bool BLEMeshInterface::isSending()
{
    NimBLEExtAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    return pAdvertising->isActive(BLE_MESH_ADV_INST_ID);
}

ErrorCode BLEMeshInterface::send(meshtastic_MeshPacket *p)
{
    assert(p);
    ErrorCode res = txQueue.enqueue(p) ? ERRNO_OK : ERRNO_UNKNOWN;
    if (res != ERRNO_OK) { // we weren't able to queue it, so we must drop it to prevent leaks
        packetPool.release(p);
        return res;
    }

    uint32_t delay = 1;
    if (p->tx_after) {
        delay = p->tx_after - millis();
    }

    notifyLater(delay, TRANSMIT_DELAY_COMPLETED, false);
    return res;
}

float BLEMeshInterface::getFreq()
{
    LOG_WARN("BLEMeshInterface getFreq");
    return 2400000;
};

void BLEMeshInterface::saveFreq(float savedFreq)
{
    LOG_WARN("BLEMeshInterface saveFreq %f", savedFreq);
}

void BLEMeshInterface::saveChannelNum(uint32_t savedChannelNum)
{
    LOG_WARN("BLEMeshInterface saveChannelNum %u", savedChannelNum);
}

bool BLEMeshInterface::sendBLE(uint8_t *data, size_t len, int max_events)
{
    // Service Data max paylaod is 237
    if (len > 237) {
        LOG_ERROR("BLEMeshInterface sendBLE paylaod to long %d", len);
        return false;
    }

    NimBLEExtAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    if (pAdvertising->isActive(BLE_MESH_ADV_INST_ID)) {
        LOG_ERROR("BLEMeshInterface sendBLE still sending last");
        return false;
    }

    NimBLEExtAdvertisement leMeshAdvertising(BLE_HCI_LE_PHY_CODED, BLE_HCI_LE_PHY_CODED);
    leMeshAdvertising.setServiceData(NimBLEUUID(MESH_RF_UUID), data, len);
    leMeshAdvertising.setAnonymous(true);
    leMeshAdvertising.setConnectable(false);
    leMeshAdvertising.setScannable(false);
    leMeshAdvertising.setMinInterval(32);
    leMeshAdvertising.setMaxInterval(50);
    leMeshAdvertising.setTxPower(15); // max for ESP32 is 20

    if (!pAdvertising->setInstanceData(BLE_MESH_ADV_INST_ID, leMeshAdvertising)) {
        LOG_ERROR("BLE failed to set leMeshAdvertising dataLen: %d advLen: %d", len, leMeshAdvertising.getDataSize());
        return false;
    }

    if (!pAdvertising->start(BLE_MESH_ADV_INST_ID, 0, max_events)) {
        LOG_ERROR("BLE failed to start leMeshAdvertising");
        return false;
    }

    LOG_WARN("BLE TX started");
    return true;
}

void BLEMeshInterface::onNotify(uint32_t notification)
{
    LOG_WARN("BLEMeshInterface onNotify %d", notification);
    switch (notification) {
    case BLE_SCAN_STOPPED:
        NimBLEDevice::getScan()->start(0, false, true);
        break;

    case TRANSMIT_DELAY_COMPLETED:
        if (isSending() || sendingPacket != NULL) {
            LOG_DEBUG("BLEMeshInterface still sending old package");
            notifyLater(100, TRANSMIT_DELAY_COMPLETED, false);
            break;
        }
        if (!txQueue.empty()) {
            meshtastic_MeshPacket *txp = txQueue.getFront();
            assert(txp);
            uint32_t delay_remaining = txp->tx_after ? txp->tx_after - millis() : 0;
            if (delay_remaining > 0) {
                notifyLater(delay_remaining, TRANSMIT_DELAY_COMPLETED, false);
                return;
            }

            txp = txQueue.dequeue();
            assert(txp);
            LOG_WARN("BLEMeshInterface send triggered");
            printPacket("BLE Lora TX", txp);

            size_t numbytes = beginSending(txp);
            powerMon->setState(meshtastic_PowerMon_State_Lora_TXOn);
            if (!sendBLE((uint8_t *)&radioBuffer, numbytes, 12)) {
                auto txp = sendingPacket;
                sendingPacket = NULL;
                if (txp) {
                    printPacket("Failed sending", txp);
                    packetPool.release(txp);
                }
                powerMon->clearState(meshtastic_PowerMon_State_Lora_TXOn);
                notifyLater(100, TRANSMIT_DELAY_COMPLETED, false);
            };
        }
        break;
    }
}

#endif