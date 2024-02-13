#define F_CPU 16000000UL // กำหนดความถี่ของไมโครคอนโทรลเลอร์ในหน่วย Hz (16 MHz)

#include <avr/io.h>
#include <util/delay.h>

#define rs PD2 // กำหนดขาที่ใช้ RS (Register Select)
#define en PD3 // กำหนดขาที่ใช้ EN (Enable)

#define BUTTON_PIN 13 // กำหนดขา Button คือขา 13/PIN PB5

int TRIG_PIN = 9; // กำหนดขา Trigger คือขา 9/PIN PB1
int ECHO_PIN = 10; // กำหนดขา Echo คือขา 10/PIN PB2

int SERVO_PIN = 8; // กำหนดขา Servo Motor คือขา 8/PIN PB0

int currentState = 0;
int previousState = 0;
int pressState = 0;

int count = 0;
char count_str[50];
long duration,distance;
char distance_str[100];

void lcd_init(void)
{
    DDRD = 0xFF;
    _delay_ms(15);  // Wait for the LCD to power up

    // Initialization sequence
    lcd_command(0x02); // Return Home
    lcd_command(0x28); // 4-bit mode, 2 lines, 5x8 font
    lcd_command(0x0C); // Display on, cursor off, blink off
    lcd_command(0x06); // Increment cursor
    lcd_command(0x01); // Clear display
    _delay_ms(2);      // Wait for the clear display command to complete
}

void lcd_command(unsigned char cmd)
{
    PORTD = (PORTD & 0x0F) | (cmd & 0xF0);    // ตั้งค่า PORTD ให้ D0-D3 ไม่เปลี่ยนแปลง, D4-D7 ให้เท่ากับ 4 บิตบนที่มีค่าจาก cmd เพื่อส่ง 4 บิตบนของ cmd ไปที่ LCD 
    PORTD &= ~(1 << rs);                      // ปิด RS (ให้ขา PD2 เป็น Low) เลือกว่าจะส่งคำสั่งควบคุม (control) ไปยัง LCD
    PORTD |= (1 << en);                       // เปิด EN (ให้ขา PD3 เป็น High) เพื่อส่งคำสั่งหรือข้อมูลไปยัง LCD
    _delay_us(1); 
    PORTD &= ~(1 << en);                      // ปิด EN (ให้ขา PD3 เป็น Low) เพื่อสิ้นสุดการส่งคำสั่งหรือข้อมูล
    _delay_us(200);
    PORTD = (PORTD & 0x0F) | (cmd << 4);      // ตั้งค่า PORTD ให้ D0-D3 เท่ากับ 4 บิตที่มีค่าจาก cmd เพื่อส่ง 4 บิตล่างของ cmd ไปที่ LCD
    PORTD |= (1 << en);                       // เปิด EN (ให้ขา PD3 เป็น High) เพื่อส่งคำสั่งหรือข้อมูลไปยัง LCD
    _delay_us(1);
    PORTD &= ~(1 << en);                      // ปิด EN (ให้ขา PD3 เป็น Low) เพื่อสิ้นสุดการส่งคำสั่งหรือข้อมูล
    _delay_ms(2);
}

void lcd_data(unsigned char data)
{
    PORTD = (PORTD & 0x0F) | (data & 0xF0);   // ตั้งค่า PORTD ให้ D0-D3 ไม่เปลี่ยนแปลง, D4-D7 ให้เท่ากับ 4 บิตบนที่มีค่าจาก data เพื่อส่ง 4 บิตบนของ data ไปที่ LCD 
    PORTD |= (1 << rs);                       // เปิด RS (ให้ขา PD2 เป็น High) เลือกว่าจะส่งข้อมูล (data) ไปยัง LCD
    PORTD |= (1 << en);                       // เปิด EN (ให้ขา PD3 เป็น High) เพื่อส่งคำสั่งหรือข้อมูลไปยัง LCD
    _delay_us(1);
    PORTD &= ~(1 << en);                      // ปิด EN (ให้ขา PD3 เป็น Low) เพื่อสิ้นสุดการส่งคำสั่งหรือข้อมูล
    _delay_us(200);
    PORTD = (PORTD & 0x0F) | (data << 4);     // ตั้งค่า PORTD ให้ D0-D3 เท่ากับ 4 บิตที่มีค่าจาก data เพื่อส่ง 4 บิตล่างของ data ไปที่ LCD
    PORTD |= (1 << en);                       // เปิด EN (ให้ขา PD3 เป็น High) เพื่อส่งคำสั่งหรือข้อมูลไปยัง LCD
    _delay_us(1);
    PORTD &= ~(1 << en);                      // ปิด EN (ให้ขา PD3 เป็น Low) เพื่อสิ้นสุดการส่งคำสั่งหรือข้อมูล
    _delay_ms(2);
}

void lcd_print(const char *str)
{
    while (*str)
    {
        lcd_data(*str++);
    }
}

void lcd_clear()
{
    lcd_command(0x01); // Clear display
    _delay_ms(2);
    lcd_command(0x80); // Clear display
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t position = 0x80; // ตำแหน่งเริ่มต้นในการ set cursor ที่หน้าจอ
    if (row == 1) {
        position = 0xC0; // บรรทัดที่ 2
    }
    position += col; // เลื่อน cursor ไปที่คอลัมน์ที่กำหนด

    lcd_command(position);
}

void setup() {
  Serial.begin(9600);

  DDRB &= ~(1 << PB5); // Set BUTTON_PIN as INPUT
  DDRB |= (1 << PB0); // Set SERVO_PIN as OUTPUT
  
  //Set Ultrasonic Sensor OUT/IN
  DDRB |= (1 << PB1);  // Set TRIG_PIN as OUTPUT
  DDRB &= ~(1 << PB2); // Set ECHO_PIN as INPUT

  lcd_clear();
  lcd_init();
  lcd_set_cursor(0, 5);
  lcd_print("Welcome");
  lcd_set_cursor(1, 3);
  lcd_print("Jaaaaaaaaaa");
  
  PORTB |= (1 << PB0); // Set Servo status High
  delayMicroseconds(map(90, 0, 90, 0, 700)); // Val = 90 Scale 0-90 degrees to 0-700 microseconds(ปิด)
  PORTB &= ~(1 << PB0); // Set Servo status LOW
}

void loop(){
  checkbutton();
  Start();
}

void checkbutton(){
  if(digitalRead(BUTTON_PIN) == HIGH) {
    // Perform actions here
    pressState += 1;
  }
}

void Start(){
  if(pressState >= 1){
    // ส่งสัญญาณ Trig เป็นช่วงเวลา 10μs
    PORTB |= (1 << PB1); // Set Trig status High
  	_delay_us(10);
  	PORTB &= ~(1 << PB1); // Set Trig status LOW
  
  	// รอรับค่าเวลาจาก Echo
  	duration = pulseIn(ECHO_PIN, HIGH); // รอให้ ECHO_PIN ไปที่ HIGH แล้วส่งกลับระยะเวลาเป็นไมโครวินาที
  
  	// คำนวณความยาวในระยะเซนติเมตร
  	distance = (duration / 58);

  	// แสดงผลลัพธ์ลงบน LCD
  	lcd_set_cursor(0, 0);
  	lcd_print("Distance: ");
  
  	// เมื่อระยะที่ตรวจจับเท่ากับหรือน้อยกว่า 50 cm ให้ servo motor หมุนขึ้น
  	if (distance <= 50) {
      itoa(distance, distance_str, 10);
      lcd_print(distance_str);
      lcd_print(" cm   ");

      currentState = 1;
      
      PORTB |= (1 << PB0); // Set Servo status High
      delayMicroseconds(map(90, 0, 90, 0, 1500)); // Val = 90 Scale 0-90 degrees to 0-1500 microseconds(เปิด)
      PORTB &= ~(1 << PB0); // Set Servo status LOW
    }
    
    // เมื่อระยะที่ตรวจจับเท่ากับหรือมากกว่า 50 cm ให้ servo motor หมุนลง
    else{
      //lcd_print("       ");
      itoa(distance, distance_str, 10);
      lcd_print(distance_str);
      lcd_print(" cm");

      delay(2000);

      PORTB |= (1 << PB0); // Set Servo status High
      delayMicroseconds(map(90, 0, 90, 0, 700)); // Val = 90 Scale 0-90 degrees to 0-700 microseconds(ปิด)
      PORTB &= ~(1 << PB0); // Set Servo status LOW
      if(currentState == 1){
        currentState = 0;
        count = count + 1;
      }
    }
    lcd_set_cursor(1,0);
    lcd_print("Car = ");

    itoa(count, count_str, 10);
    lcd_print(count_str);
    lcd_print("           ");

    // รอสักครู่ก่อนที่จะทำการวนลูปใหม่
    delay(200);
  
    previousState = currentState;
    }
}