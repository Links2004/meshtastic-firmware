#pragma once
#if !MESHTASTIC_EXCLUDE_BLUETOOTH && defined(CONFIG_BT_NIMBLE_EXT_ADV) && RADIOLIB_EXCLUDE_BLE_MESH_RF != 1 && defined(NIMBLE_TWO)
#include "RadioLibInterface.h"
#include "nimble/NimbleBluetooth.h"

#include "concurrency/NotifiedWorkerThread.h"

#include "NimBLEDevice.h"
#include "NimBLEExtAdvertising.h"
#include "NimBLEScan.h"

#define BLE_MESH_ADV_INST_ID 1

class BLEMeshInterface : public RadioInterface,
                         public NimBLEScanCallbacks,
                         public NimBLEExtAdvertisingCallbacks,
                         protected concurrency::NotifiedWorkerThread
{
  public:
    BLEMeshInterface(NimbleBluetooth *nimbleBluetooth);

    enum PendingISR { ISR_NONE = 0, TRANSMIT_DELAY_COMPLETED, BLE_SCAN_STOPPED };

    /// Initialise the Driver transport hardware and software.
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool init() override;

    /// Apply any radio provisioning changes
    /// Make sure the Driver is properly configured before calling init().
    /// \return true if initialisation succeeded.
    virtual bool reconfigure() override;

    /**
     * Send a packet (possibly by enquing in a private fifo).  This routine will
     * later free() the packet to pool.  This routine is not allowed to stall.
     * If the txmit queue is full it might return an error
     */
    virtual ErrorCode send(meshtastic_MeshPacket *p) override;

    /**
     * Get the frequency we saved.
     */
    virtual float getFreq() override;

    virtual bool canSleep() override;

    virtual bool wideLora() override { return true; }

    meshtastic_QueueStatus getQueueStatus();

    /** Attempt to cancel a previously sent packet.  Returns true if a packet was found we could cancel */
    virtual bool cancelSending(NodeNum from, PacketId id) override;

    /** Attempt to find a packet in the TxQueue. Returns true if the packet was found. */
    virtual bool findInTxQueue(NodeNum from, PacketId id) override;

    void bleRX(meshtastic_MeshPacket *mp);

    // BLE RX
    void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override;
    void onScanEnd(const NimBLEScanResults &results, int reason) override;

    // BLE TX
    void onStopped(NimBLEExtAdvertising *pAdv, int reason, uint8_t instId) override;

    // OSThread
    void onNotify(uint32_t notification);

  protected:
    /*
     * Save the frequency we selected for later reuse.
     */
    virtual void saveFreq(float savedFreq) override;

    /**
     * Save the channel we selected for later reuse.
     */
    virtual void saveChannelNum(uint32_t savedChannelNum) override;

    bool isSending();

  private:
    NimbleBluetooth *nimbleBluetooth = nullptr;
    RadioBuffer rxRadioBuffer __attribute__((__aligned__));

    MeshPacketQueue txQueue = MeshPacketQueue(MAX_TX_QUEUE);

    bool sendBLE(uint8_t *data, size_t len, int max_events);

    float getNoiseFloor();
};

#endif