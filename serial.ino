#define USE_INTERNAL_PIN_LOOPBACK 1   // 1 uses the internal loopback, 0 for wiring pins 4 and 5 externally

#define DATA_SIZE 26    // 26 bytes is a lower than RX FIFO size (127 bytes) 
#define BAUD 115200     // Any baudrate from 300 to 115200
#define TEST_UART 1     // Serial1 will be used for the loopback testing with different RX FIFO FULL values
#define RXPIN 4         // GPIO 4 => RX for Serial1
#define TXPIN 5         // GPIO 5 => TX for Serial1

String inString;

void serial_setup() {
    // UART0 will be used to log information into Serial Monitor
  Serial.begin(115200);

  Serial1.begin(BAUD, SERIAL_8N1, RXPIN, TXPIN); // Rx = 4, Tx = 5 will work for ESP32, S2, S3 and C3
#if USE_INTERNAL_PIN_LOOPBACK
  uart_internal_loopback(TEST_UART, RXPIN);
#endif
}

void serial_read() {
    while (Serial1.available() > 0)
    {
        char recieved = Serial1.read();
        
        if (recieved == '\n')
        {              
            for(int i=0; i<=inString.length(); i++) {
              if (inString[i] == 'A') {
                adcValue = inString[i+1];
              }
              else if (inString[i] == 'B') {
                dhtTValue = inString[i+1];
                dhtHValue = inString[i+2];
              }
              else if (inString[i] == 'C') {
                btnValue = inString[i+1];
              }
            }
  
            inString = ""; // Clear recieved buffer
            break;
        }
        inString += recieved;
        valuesUpdated = true;
    }
}

void serial_write() {
  Serial1.write('D');
  Serial1.write((char)ledValue);
  Serial1.write('\n');
}
