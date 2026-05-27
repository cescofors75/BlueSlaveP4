import struct
with open('.pio/build/esp32p4/bootloader.bin','rb') as f:
    data = f.read()
print('First 32 bytes:', ' '.join(f'{b:02x}' for b in data[:32]))
magic = data[0]
segments = data[1]
spi_mode = data[2]
entry = struct.unpack('<I', data[16:20])[0]
print(f'Magic: {magic:#x}, Segments: {segments}, SPI mode: {spi_mode:#x}, Entry: {entry:#x}')
offset = 24
for i in range(segments):
    load_addr = struct.unpack('<I', data[offset:offset+4])[0]
    length = struct.unpack('<I', data[offset+4:offset+8])[0]
    seg_data = data[offset+8:offset+8+length]
    print(f'Segment {i}: load_addr={load_addr:#x}, length={length:#x}')
    if load_addr == 0x4ffac2c0:
        print(f'  Entry point segment! First 16 bytes: {" ".join(f"{b:02x}" for b in seg_data[:16])}')
        for j in range(0, min(32, length), 4):
            insn = struct.unpack('<I', seg_data[j:j+4])[0]
            print(f'  +{j:#04x}: {insn:#010x}')
    offset += 8 + length
