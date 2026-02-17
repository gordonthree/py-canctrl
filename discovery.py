import can
import struct
import time

# CRC-16/CCITT-FALSE (ESP32 rom/crc.h crc16_be)
def crc16_be(data: bytes, initial=0xFFFF):
    crc = initial
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

class CANManager:
    def __init__(self):
        self.nodes = {}
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')

    def calculate_node_crc(self, node_id: int):
        nd = self.nodes.get(node_id)
        if not nd: return 0
        
        # 136 bytes: 8 submodules (16 bytes each) + 8 bytes metadata
        buf = bytearray(136)
        
        for i in range(8):
            offset = i * 16
            if i in nd['subs']:
                s = nd['subs'][i]
                # [0:3] config.rawConfig
                if s['cfg']:
                    buf[offset:offset+3] = s['cfg']
                # [3] reserved
                buf[offset+3] = 0x00
                # [4:8] data union (operational, set to 0 for config CRC)
                buf[offset+4:offset+8] = b'\x00\x00\x00\x00'
                # [8:10] introMsgId (Little Endian uint16)
                struct.pack_into('<H', buf, offset + 8, s.get('intro_id', 0))
                # [10:12] dataMsgId (Little Endian uint16)
                if s['telemetry']:
                    struct.pack_into('<H', buf, offset + 10, s['telemetry']['id'])
                # [12] introMsgDLC
                buf[offset + 12] = s.get('intro_dlc', 0)
                # [13] dataMsgDLC
                if s['telemetry']:
                    buf[offset + 13] = s['telemetry']['dlc']
                # [14] saveState
                if s['telemetry']:
                    buf[offset + 14] = 1 if s['telemetry']['save'] else 0
                # [15] padding
                buf[offset + 15] = 0x00
        
        # Metadata at end of 136-byte block
        # [128:132] nodeID (Little Endian uint32)
        struct.pack_into('<I', buf, 128, node_id)
        # [132:134] nodeTypeMsg (Little Endian uint16)
        struct.pack_into('<H', buf, 132, nd.get('node_type_msg', 0))
        # [134] nodeTypeDLC
        buf[134] = nd.get('node_type_dlc', 0)
        # [135] subModCnt
        buf[135] = nd.get('sub_mod_cnt', 0)

        # Write debug dump for comparison
        filename = f"node_{hex(node_id)}_buffer.bin"
        with open(filename, "wb") as f:
            f.write(buf)
        
        # Return results for both common initializations
        return {
            'ffff': crc16_be(buf, 0xFFFF),
            '0000': crc16_be(buf, 0x0000),
            'dump_file': filename
        }

    def process_queue(self):
        msg = self.bus.recv(timeout=1.0)
        if not msg: return

        # MsgPtr extraction
        m_ptr = msg.data[0]
        m_byte = msg.data[1]
        node_id = struct.unpack('<I', msg.data[0:4])[0] if m_ptr == 0 else 0 
        # Note: If m_ptr != 0, we need a lookup for which node is being interviewed.
        # This implementation assumes we are tracking the active interview ID.
        
        if m_ptr == 0:
            # Identity Frame (Introduction)
            self.active_id = node_id
            self.nodes[node_id] = {
                'node_type_msg': msg.arbitration_id,
                'node_type_dlc': msg.dlc,
                'sub_mod_cnt': msg.data[4],
                'reported_crc': (msg.data[5] << 8) | msg.data[6],
                'subs': {}
            }
            print(f"Discovered Node: {hex(node_id)} | Expected CRC: {hex(self.nodes[node_id]['reported_crc'])}")

        elif m_ptr > 0 and hasattr(self, 'active_id'):
            # Submodule interview frames
            node = self.nodes[self.active_id]
            idx, is_b = m_byte & 0x7F, bool(m_byte & 0x80)
            
            if idx not in node['subs']:
                node['subs'][idx] = {'cfg': None, 'telemetry': None, 'intro_id': 0, 'intro_dlc': 0}
            
            if not is_b: # Part A
                node['subs'][idx]['cfg'] = msg.data[2:5]
                node['subs'][idx]['intro_id'] = msg.arbitration_id
                node['subs'][idx]['intro_dlc'] = msg.dlc
            else: # Part B
                node['subs'][idx]['telemetry'] = {
                    'id': (msg.data[5] << 8) | msg.data[6],
                    'dlc': msg.data[7] & 0x0F,
                    'save': bool(msg.data[7] & 0x80)
                }
                
                # Check CRC if this was the last submodule
                if len(node['subs']) == node['sub_mod_cnt']:
                    res = self.calculate_node_crc(self.active_id)
                    print(f"CRC Check for {hex(self.active_id)}:")
                    print(f"  Reported: {hex(node['reported_crc'])}")
                    print(f"  Python (Init 0xFFFF): {hex(res['ffff'])}")
                    print(f"  Python (Init 0x0000): {hex(res['0000'])}")
                    print(f"  Buffer dump saved to: {res['dump_file']}")

if __name__ == "__main__":
    mgr = CANManager()
    while True:
        mgr.process_queue()