import serial

PORT      = "COM13"
BAUDRATE  = 115200
CSV_FILE  = "rssi_data.csv"

print(f"Monitoring CSV output — saving to {CSV_FILE}")
print("Press Ctrl+C to stop\n")

with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
    with open(CSV_FILE, 'w') as f:
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith('_id') or line.startswith('esp32_tag'):
                    print(line)
                    f.write(line + '\n')
                    f.flush()
            except KeyboardInterrupt:
                print(f"\nStopped. Data saved to {CSV_FILE}")
                break