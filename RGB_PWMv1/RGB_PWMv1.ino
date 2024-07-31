int red = 9; //10
int green = 10; //9
int blue = 6;

int redValue;
int greenValue;
int blueValue;

#define delayTime 100 // fading time between colors
#define maxBrightness 255 // max possible is 255

void setup() {
//  Serial.begin(9600);
pinMode(red, OUTPUT);
pinMode(green, OUTPUT);
pinMode(blue, OUTPUT);
digitalWrite(red, HIGH);
digitalWrite(green, LOW);
digitalWrite(blue, LOW);

delay(1000);
digitalWrite(red, LOW);
digitalWrite(green, HIGH);
digitalWrite(blue, LOW);

delay(1000);
digitalWrite(red, LOW);
digitalWrite(green, LOW);
digitalWrite(blue, HIGH);

delay(1000);
digitalWrite(red, HIGH);
digitalWrite(green, LOW);
digitalWrite(blue, LOW);

}

void loop() {
for(int i = 0; i < maxBrightness; i+=1) // fades out red bring green full when i = 255
  {
    redValue -= 1;
    greenValue += 1;
    analogWrite(red,redValue);
    analogWrite(green, greenValue);
    delay(delayTime);
  }
//  Serial.println("green");
  redValue = 0;
  greenValue = 255;
  blueValue = 0;

  for(int i = 0; i < maxBrightness; i +=1) // fades out green bring blue full when i = 255
  {
    greenValue -= 1;
    blueValue += 1;
    analogWrite(green, greenValue);
    analogWrite(blue, blueValue);
    delay(delayTime);
  }

// Serial.println("blue");
  redValue = 0;
  greenValue = 0;
  blueValue = 255;
  
   for(int i = 0; i < maxBrightness; i +=1) // fades out blue bring red full when i = 255
  {
    blueValue -= 1;
    redValue += 1;
    analogWrite(red, redValue);
    analogWrite(blue, blueValue);
    delay(delayTime);
  } 
//  Serial.println("red");
  redValue = 255;
  greenValue = 0;
  blueValue = 0;
}
