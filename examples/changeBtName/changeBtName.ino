static const char* name = "CAM-016"; // Max 20 characters!
static const char* pin = "1234"; // Strict 4 digits!



class HC06 {
public:
    HC06(Print* s) : s_(s) {}

    bool setBaudRate(uint32_t baud)
    {
        const char c = letter(baud);
        if (c == '\0') {
            return false;
        }
        s_->print(F("AT+BAUD"));
        s_->print(c);
        return true;
    }

    bool setName(const char* name)
    {
        s_->print(F("AT+NAME"));
        s_->print(name);
        return true;
    }

    bool setPin(const char* pin)
    {
        s_->print(F("AT+PIN"));
        s_->print(pin);
        return true;
    }

private:
    char letter(uint32_t baud)
    {
        switch (baud) {
        case 1200u:
            return '1';
        case 2400u:
            return '2';
        case 4800u:
            return '3';
        case 9600u:
            return '4';
        case 19200u:
            return '5';
        case 38400u:
            return '6';
        case 57600u:
            return '7';
        case 115200u:
            return '8';
        case 230400u:
            return '9';
        case 460800u:
            return 'A';
        case 921600u:
            return 'B';
        case 1382400u:
            return 'C';
        default:
            s_->print(F("unsupported baud rate"));
            return '\0';
        }
    }

    Print* s_;
};

HC06 hc{&Serial};

void setup() {
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.begin(9600);
  delay(2000);
  hc.setName(name);
  delay(2000);
  hc.setPin(pin);
  delay(2000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  
}
