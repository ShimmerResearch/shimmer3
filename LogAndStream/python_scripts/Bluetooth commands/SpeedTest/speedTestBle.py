
import time
from pc_ble_driver_py.observers import *
from queue import *
from threading import Thread
import logging

CONNECTIONS = 1
CFG_TAG = 1

# Select DEBUG_BLE_MESSAGES = True to output out all BLE configuration steps to Console/Logger file
DEBUG_BLE_MESSAGES = True

TARGET_MAC_ID = "A615"

logging.basicConfig(level=logging.INFO,
                    format='%(asctime)-12s %(message)s',
                    datefmt='%m-%d %H:%M',
                    # filename=logger_path,
                    filemode='w')

logger = logging.getLogger('root')


def init(conn_ic_id):
    # noinspection PyGlobalUndefined
    global config, BLEDriver, BLEAdvData, BLEEvtID, BLEAdapter, BLEEnableParams, BLEGapTimeoutSrc, BLEUUID, BLEUUIDBase, BLEConfigCommon, BLEConfig, BLEConfigConnGatt, BLEGapScanParams, BLEConfigConnGap, BLEGapConnParams, BLEGapIOCaps, BLEGapSecStatus, driver, util
    from pc_ble_driver_py import config

    config.__conn_ic_id__ = conn_ic_id
    # noinspection PyUnresolvedReferences
    from pc_ble_driver_py.ble_driver import (
        BLEDriver,
        BLEAdvData,
        BLEEvtID,
        BLEEnableParams,
        BLEGapTimeoutSrc,
        BLEUUID,
        BLEUUIDBase,
        BLEGapScanParams,
        BLEGapConnParams,
        BLEConfigCommon,
        BLEConfig,
        BLEConfigConnGatt,
        BLEConfigConnGap,

        # Added by Shimmer
        # BLEGapSecParams,
        BLEGapIOCaps,
        # BLEGapSecKDist,
        # BLEGapRoles,
        BLEGapSecStatus,
        # BLEGapSecKeyset

        driver,
        util,
    )

    # noinspection PyUnresolvedReferences
    from pc_ble_driver_py.ble_adapter import BLEAdapter

    global nrf_sd_ble_api_ver
    nrf_sd_ble_api_ver = config.sd_api_ver_get()

    # import pc_ble_driver_py.ble_driver_types as util


class NUSParser(BLEDriverObserver, BLEAdapterObserver):
    def __init__(self, adapter):
        super(NUSParser, self).__init__()
        self.adapter = adapter
        self.conn_q = Queue()
        self.adapter.observer_register(self)
        self.adapter.driver.observer_register(self)
        self.adapter.default_mtu = 247

        # # UUID associated with the Nordic UART Service
        # self.nus_base = BLEUUIDBase([0x6E, 0x40, 0x00, 0x00, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E])
        # self.nus_rx = BLEUUID(0x0002, self.nus_base)
        # self.nus_tx = BLEUUID(0x0003, self.nus_base)

        # UUID associated with the Roving Networks UART Service
        # 49535341 6DAA 4D02 ABF6 19569ACA69FE
        self.nus_base = BLEUUIDBase([0x49, 0x53, 0x53, 0x43, 0xFE, 0x7D, 0x4A, 0xE5, 0x8F, 0xA9, 0x9F, 0xAF, 0xD2, 0x05, 0xE4, 0x55])
        # self.nus_rx = BLEUUID([0x49, 0x53, 0x53, 0x43, 0x1E, 0x4D, 0x4B, 0xD9, 0xBA, 0x61, 0x23, 0xC6, 0x47, 0x24, 0x96, 0x16], self.nus_base)
        # self.nus_tx = BLEUUID([0x49, 0x53, 0x53, 0x43, 0x88, 0x41, 0x43, 0xF4, 0xA8, 0xD4, 0xEC, 0xBE, 0x34, 0x72, 0x9B, 0xB3], self.nus_base)
        self.nus_rx = BLEUUID(0x9616, self.nus_base)
        self.nus_tx = BLEUUID(0x9BB3, self.nus_base)

    def open(self):
        self.adapter.driver.open()
        if config.__conn_ic_id__ == "NRF51":
            self.adapter.driver.ble_enable(
                BLEEnableParams(
                    vs_uuid_count=1,
                    service_changed=0,
                    periph_conn_count=0,
                    central_conn_count=CONNECTIONS,
                    central_sec_count=0,
                )
            )
        elif config.__conn_ic_id__ == "NRF52":
            gatt_cfg = BLEConfigConnGatt()
            gatt_cfg.att_mtu = self.adapter.default_mtu
            gatt_cfg.tag = CFG_TAG
            self.adapter.driver.ble_cfg_set(BLEConfig.conn_gatt, gatt_cfg)

            conn_cfg = BLEConfigConnGap()
            conn_cfg.conn_count = CONNECTIONS
            conn_cfg.event_length = 320
            self.adapter.driver.ble_cfg_set(BLEConfig.conn_gap, conn_cfg)

            self.adapter.driver.ble_enable()
            self.adapter.driver.ble_vs_uuid_add(self.nus_base)

    def close(self):
        self.adapter.driver.close()

    def connect_and_discover(self):
        scan_duration = 1100
        scan_params = BLEGapScanParams(interval_ms=200, window_ms=150, timeout_s=scan_duration)
        new_conn = None

        # i = 0
        # while 1:
        #     if i > 0:
        #         logger.info("\nPasskey rejected: Beginning Connection Retry no. {}" .format(i))
        #         try:
        #             self.adapter.disconnect(new_conn)
        #             self.adapter.evt_sync[new_conn].wait(evt=BLEEvtID.gap_evt_disconnected)
        #         except:
        #             logger.info("\nException disconnecting")
        #         time.sleep(1)
        #
        #     try:
        #         # Scan and connect
        #         self.adapter.driver.ble_gap_scan_start(scan_params)
        #         new_conn = self.conn_q.get(timeout=scan_duration)
        #
        #         # Authenticate if requested to
        #         ASM_Device.passkey_enabled = False
        #         auth_success = False
        #         # Wait for security request. Note, this adds an unnecessary delay when connecting to sensors that don't have passkey set.
        #         result = self.adapter.evt_sync[new_conn].wait(evt=BLEEvtID.gap_evt_sec_request)
        #         if result is not None and 'bond' in result and result['bond'] == 1:
        #             ASM_Device.passkey_enabled = True
        #
        #             Thread(
        #                 target=self.adapter.authenticate,
        #                 args=(new_conn, None),
        #                 kwargs={"bond": True, "mitm": True, "io_caps": BLEGapIOCaps.keyboard_only},
        #             ).start()
        #
        #             result = self.adapter.evt_sync[new_conn].wait(evt=BLEEvtID.gap_evt_auth_status)
        #
        #             auth_success = result is not None and "auth_status" in result and result["auth_status"] == BLEGapSecStatus.success
        #
        #         # If passkey not set or authorisation has been successful, perform service discovery
        #         if not ASM_Device.passkey_enabled or auth_success:
        #             # Update PHY to 2MBPS
        #             req_phys = [driver.BLE_GAP_PHY_2MBPS, driver.BLE_GAP_PHY_2MBPS]
        #             logging.info(f"Central requesting: req_phys={req_phys}")
        #             resp = self.adapter.phy_update(new_conn, req_phys)
        #             result = self.adapter.evt_sync[new_conn].wait(evt=BLEEvtID.gap_evt_data_length_update)
        #
        #             self.adapter.service_discovery(new_conn)
        #             if DEBUG_BLE_MESSAGES:
        #                 logger.info('Service Discovery:')
        #                 for s in self.adapter.db_conns[new_conn].services:
        #                     logger.info('\t{}'.format(s))
        #                     for ch in s.chars:
        #                         logger.info('\t\t{}'.format(ch))
        #
        #             # Nordic UART Service (NUS)
        #             self.adapter.enable_notification(conn_handle=new_conn, uuid=self.nus_tx)
        #
        #             break
        #     except:
        #         logger.info("\nException connecting")
        #
        #     i += 1

        self.adapter.driver.ble_gap_scan_start(scan_params)
        new_conn = self.conn_q.get(timeout=scan_duration)

        # Update PHY to 2MBPS
        req_phys = [driver.BLE_GAP_PHY_2MBPS, driver.BLE_GAP_PHY_2MBPS]
        logging.info(f"Central requesting: req_phys={req_phys}")
        resp = self.adapter.phy_update(new_conn, req_phys)
        result = self.adapter.evt_sync[new_conn].wait(evt=BLEEvtID.gap_evt_data_length_update)

        self.adapter.service_discovery(new_conn)
        if DEBUG_BLE_MESSAGES:
            logger.info('Service Discovery:')
            for s in self.adapter.db_conns[new_conn].services:
                logger.info('\t{}'.format(s))
                for ch in s.chars:
                    logger.info('\t\t{}'.format(ch))

        # UART Service
        self.adapter.enable_notification(conn_handle=new_conn, uuid=self.nus_tx)

        return new_conn

    def scan_and_check(self):
        # This function is used in ASM_PendingEventsTest.py to perform a scan to confirm that the BLE has successfully
        # Turned off once the Python has disconnected from the device.
        self.adapter.driver.ble_gap_scan_start()
        new_conn = None
        try:
            new_conn = self.conn_q.get(timeout=30)
            return new_conn
        except Empty:
            self.close()
            time.sleep(3)
            return new_conn

    def on_gap_evt_connected(self, ble_driver, conn_handle, peer_addr, role, conn_params):
        if DEBUG_BLE_MESSAGES:
            logger.info('GAP_EVT->\tConnected: {}, {}'.format(conn_handle, conn_params))
        self.conn_q.put(conn_handle)

    def on_gap_evt_disconnected(self, ble_driver, conn_handle, reason):
       if DEBUG_BLE_MESSAGES:
           logger.info('GAP_EVT->\tDisconnected: {} {}'.format(conn_handle, reason))

    def on_gap_evt_timeout(self, ble_driver, conn_handle, src):
       if src == BLEGapTimeoutSrc.scan:
           ble_driver.ble_gap_scan_start()

    def on_gap_evt_adv_report(self, ble_driver, conn_handle, peer_addr, rssi, adv_type, adv_data):
        conn_params = BLEGapConnParams(min_conn_interval_ms=7.5, max_conn_interval_ms=7.5, conn_sup_timeout_ms=4000, slave_latency=0)

        dev_name_list = None
        if BLEAdvData.Types.complete_local_name in adv_data.records:
            dev_name_list = adv_data.records[BLEAdvData.Types.complete_local_name]
        elif BLEAdvData.Types.short_local_name in adv_data.records:
            dev_name_list = adv_data.records[BLEAdvData.Types.short_local_name]
        else:
            return

        dev_name = "".join(chr(e) for e in dev_name_list)
        address_string = "".join("{0:02X}".format(b) for b in peer_addr.addr)
        if DEBUG_BLE_MESSAGES:
            logger.info('Received advertisement report, address: 0x{}, device_name: {}'.format(address_string, dev_name))

        if address_string.endswith(TARGET_MAC_ID):
            self.adapter.connect(address=peer_addr, conn_params=conn_params, tag=CFG_TAG)

    def on_gap_evt_phy_update(self, ble_driver, conn_handle, status, tx_phy, rx_phy):
        phy_update = {"status": status, "tx_phy": tx_phy, "rx_phy": rx_phy}

    def on_notification(self, ble_adapter, conn_handle, uuid, data):

        # global IS_STREAMING_DATA
        #
        # # check if time diff from last received packet is too large, if so, clear the buf
        # ts_ms = time.time() * 1000
        # if not self.asm_message_buffer.is_blank():
        #     if (ts_ms - self.asm_message_buffer.previous_transfer_ts_ms) > Timeout_ms.BETWEEN_PACKETS:
        #         logger.info('Time diff too large from previous message, starting new message')
        #         self.asm_message_buffer = AsmMessage()
        #
        # self.asm_message_buffer.add_packet(data, ts_ms)
        #
        # if self.asm_message_buffer.is_payload_header_parsed and len(self.asm_message_buffer.payload) > self.asm_message_buffer.payload_length:
        #     logger.info('WARNING, Too many bytes received - resetting buffer')
        #     self.asm_message_buffer = AsmMessage()
        # elif self.asm_message_buffer.is_full():
        #     ASM_Device.latest_transfer_speed = self.asm_message_buffer.calculate_overall_transfer_time()
        #     self.asm_message_latest = self.asm_message_buffer
        #     self.asm_message_buffer = AsmMessage()
        #     if IS_STREAMING_DATA:
        #         Shimmer_Plotting.process_data(self.asm_message_latest.payload)

        logger.info('RX')

    def on_att_mtu_exchanged(self, ble_driver, conn_handle, att_mtu):
        if DEBUG_BLE_MESSAGES:
            logger.info('ATT MTU exchanged: conn_handle={} att_mtu={}'.format(conn_handle, att_mtu))

    def on_gattc_evt_exchange_mtu_rsp(self, ble_driver, conn_handle, status, att_mtu):
        if DEBUG_BLE_MESSAGES:
            print("ATT MTU updated to {}".format(att_mtu))

    def on_gap_evt_data_length_update(self, ble_driver, conn_handle, data_length_params):
        if DEBUG_BLE_MESSAGES:
            print("Max rx octets: {}".format(data_length_params.max_rx_octets))
            print("Max tx octets: {}".format(data_length_params.max_tx_octets))
            print("Max rx time: {}".format(data_length_params.max_rx_time_us))
            print("Max tx time: {}".format(data_length_params.max_tx_time_us))

    def on_gatts_evt_exchange_mtu_request(self, ble_driver, conn_handle, client_mtu):
        print("Client requesting to update ATT MTU to {} bytes".format(client_mtu))

    def on_gap_evt_sec_params_request(self, ble_driver, conn_handle, peer_params):
        if DEBUG_BLE_MESSAGES:
            logger.info('GAP_EVT->\ton_gap_evt_sec_params_request: conn_handle={} peer_params={}'.format(conn_handle, peer_params))

    def on_gap_evt_sec_info_request(self, ble_driver, conn_handle, peer_addr, master_id, enc_info, id_info, sign_info):
        if DEBUG_BLE_MESSAGES:
            logger.info('GAP_EVT->\ton_gap_evt_sec_info_request')

    def on_gap_evt_sec_request(self, ble_driver, conn_handle, bond, mitm, lesc, keypress):
        if DEBUG_BLE_MESSAGES:
            logger.info(
                'GAP_EVT->\ton_gap_evt_sec_request: conn_handle={} bond={} mitm={} lesc={} keypress={}'.format(conn_handle, bond,
                                                                                                    mitm,
                                                                                                    lesc, keypress))

    def on_gap_evt_auth_status(self, ble_driver, conn_handle, error_src, bonded, sm1_levels, sm2_levels, kdist_own,
                              kdist_peer, auth_status):
        if DEBUG_BLE_MESSAGES:
            logger.info('GAP_EVT->\ton_gap_evt_auth_status={}'.format(auth_status))

    def on_gap_evt_auth_key_request(self, ble_driver, conn_handle, **kwargs):
        logger.info('GAP_EVT->\ton_gap_evt_auth_key_request.')

        # pk = util.list_to_uint8_array(ASM_Device.generatePasskey())
        #
        # driver.sd_ble_gap_auth_key_reply(
        #     ble_driver.rpc_adapter,
        #     conn_handle,
        #     kwargs["key_type"],
        #     pk.cast(),
        # )

   # # Based on a combination of ble_adapter.py -> authenticate and https://devzone.nordicsemi.com/f/nordic-q-a/42974/bonding-with-pc-ble-driver-py
   #  def authenticate_asm(self, conn_handle):
   #      logger.info('Pairing started')
   #
   #      kdist_own = BLEGapSecKDist(enc=1,
   #                                 id=1,
   #                                 sign=0,
   #                                 link=0)
   #      kdist_peer = BLEGapSecKDist(enc=1,
   #                                  id=1,
   #                                  sign=0,
   #                                  link=0)
   #      sec_params = BLEGapSecParams(bond=True,
   #                                   mitm=False,
   #                                   lesc=False,
   #                                   keypress=False,
   #                                   io_caps=BLEGapIOCaps.keyboard_only,
   #                                   oob=False,
   #                                   min_key_size=7,
   #                                   max_key_size=16,
   #                                   kdist_own=kdist_own,
   #                                   kdist_peer=kdist_peer)
   #
   #      self.adapter.driver.ble_gap_authenticate(conn_handle, sec_params)
   #      self.adapter.evt_sync[conn_handle].wait(evt=BLEEvtID.gap_evt_sec_params_request)
   #
   #      # sd_ble_gap_sec_params_reply ... In the central role, sec_params must be set to NULL,
   #      # as the parameters have already been provided during a previous call to
   #      # sd_ble_gap_authenticate.
   #      sec_params = None if self.adapter.db_conns[conn_handle].role == BLEGapRoles.central else sec_params
   #      self.adapter.driver.ble_gap_sec_params_reply(conn_handle, BLEGapSecStatus.success, sec_params=sec_params)
   #      if DEBUG_BLE_MESSAGES:
   #          logger.info("\tSec params replying done")
   #
   #      # Send passcode
   #      self.adapter.evt_sync[conn_handle].wait(evt=BLEEvtID.gap_evt_auth_key_request)
   #
   #      # key = util.list_to_uint8_array([0x39, 0x31, 0x34, 0x30, 0x32, 0x30]).cast()
   #      key = util.list_to_uint8_array(ASM_Device.generatePasskey()).cast()
   #      # BLE_GAP_AUTH_KEY_TYPE_PASSKEY = 0x01
   #      res = self.adapter.driver.sd_ble_gap_auth_key_reply(conn_handle, 0x01, key)
   #      if DEBUG_BLE_MESSAGES:
   #          logger.info("\tKey reply finished with code: {}".format(res))
   #
   #      # Get pairing results and save key
   #      result = self.adapter.evt_sync[conn_handle].wait(evt=BLEEvtID.gap_evt_auth_status)
   #
   #      if DEBUG_BLE_MESSAGES:
   #          logger.info("\tPairing Errors: {}".format(result['error_src']))
   #      if result['auth_status'] == BLEGapSecStatus.success:
   #          if DEBUG_BLE_MESSAGES:
   #              logger.info("\tSuccess")
   #          self.adapter.db_conns[conn_handle]._keyset = BLEGapSecKeyset.from_c(self.adapter.driver._keyset)
   #
   #      logger.info("Pairing finished")
   #
   #      # result = self.evt_sync[conn_handle].wait(evt = BLEEvtID.gap_evt_auth_status)
   #      # # If success then keys are stored in self.driver._keyset.
   #      # if result['auth_status'] ==  BLEGapSecStatus.success:
   #      #     self.db_conns[conn_handle]._keyset = BLEGapSecKeyset.from_c(self.driver._keyset)
   #      return result['auth_status']

    # MN not working
    def on_gap_evt_conn_param_update_request(self, conn_handle, conn_params):
        if DEBUG_BLE_MESSAGES:
            logger.info('on_gap_evt_conn_param_update_request={}'.format(conn_params))

    # MN not working
    def on_gap_evt_conn_param_update(self, conn_handle, conn_params):
        if DEBUG_BLE_MESSAGES:
            logger.info('EVT\ton_gap_evt_conn_param_update={}'.format(conn_params))

    def write_req(self, conn_handle, data):
        self.adapter.write_req(conn_handle=conn_handle, uuid=self.nus_rx, data=data)


class ASMCommands:

    def __init__(self, collector):
        self.collector = collector

    def speed_test_start(self, new_conn):
        logger.info('Command: Reading Status')

        data = bytearray([0xA4, 0x01])
        self.send_command(new_conn, data)
        # self.wait_for_ack(1000)

        # self.send_command_simple(new_conn, AsmCommand.READ | asm_property)
        # response = self.wait_for_response_and_check(asm_property, Timeout_ms.STANDARD)
        # if isinstance(response, bool):
        #     logger.info("FAIL: read_status")
        #     return False
        # else:
        #     ASM_Device.parse_status(response, asm_property)
        #     logger.info(space_string)
        #     return True

    def wait_for_response(self, property, timeout_ms):
        loop_count = 0
        wait_interval_ms = 100
        loop_count_total = timeout_ms / wait_interval_ms

        while True:
            time.sleep(wait_interval_ms / 1000.0)
            loop_count += 1
            if loop_count >= loop_count_total:
                return False
            # if SEND_NACK_ON_CHUNKS :
            #     if len(self.collector.asm_message_buffer.payload) > send_nack_on:
            #         return len(self.collector.asm_message_buffer.payload)
            # if self.collector.asm_message_latest.is_full():
            #     if DEBUG_BLE_TXRX_PACKETS:
            #         logger.info('Complete Response Received')
            #     return self.collector.asm_message_latest
            # # only need for the data tranmission at the moment but could be need for other reads with a large number of bytes being returned
            # elif self.collector.asm_message_buffer.property == AsmProperty.DATA:
            #     # check for 'is alive' to make sure packets are coming in before considering a timeout on the overall transmission
            #     ts_ms = time.time() * 1000
            #     ts_diff_last_msg_ms = ts_ms - self.collector.asm_message_buffer.previous_transfer_ts_ms
            #     # if the timeout is greater then the standard timeout, we'll check to see if individual packets are coming.
            #     if (timeout_ms > Timeout_ms.STANDARD) \
            #             and ((loop_count * wait_interval_ms) > Timeout_ms.STANDARD) \
            #             and ((self.collector.asm_message_buffer.previous_transfer_ts_ms == 0) or (
            #             ts_diff_last_msg_ms > Timeout_ms.STANDARD)):
            #         logger.info('FAIL: Device is possibly not responding')
            #         return False

    def wait_for_ack(self, timeout_ms):
        response = self.wait_for_response_and_check(None, timeout_ms)
        if isinstance(response, bool):
            if response:
                logger.info("ACK - End of transmission")
            else:
                logger.info("FAIL: No ACK received")
            return response


def setup():
    init("NRF52")
    serial_port = "COM45"

    driver = BLEDriver(serial_port=serial_port, auto_flash=True)
    adapter = BLEAdapter(driver)
    collector = NUSParser(adapter)

    collector.open()
    commands = ASMCommands(collector)

    for i in range(CONNECTIONS):
        conn_handle = collector.connect_and_discover()

    # collector.close()
