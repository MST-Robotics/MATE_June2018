#include <SPI.h>  
#include <Pixy.h>

#define red 1
#define blue 2
#define yellow 3
#define tolerance 1 //add a tolerance for the ratio of the object to counteract camera inaccuracy
#define rectangle_ratio 6.666 //predetermined ratios of shapes on the wings
#define triangle_ratio 1.666

//main Pixy object 
Pixy pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  pixy.init();
}

void loop()
{
  static int frame = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  
  int color; //signature determined by camera
  float width;
  float height;
  float ratio;
  float temp; //for swapping height and width
  
  // grab blocks
  blocks = pixy.getBlocks();
  
  if (blocks)
  {
    frame++;
    
    // run every 50 frames
    if (frame%50==0)
    {
      for (j=0; j<blocks; j++)
      { 
        color = pixy.blocks[j].signature;
        width = pixy.blocks[j].width;
        height = pixy.blocks[j].height;

        //switch height and width if the wing is sideways so ratios are still accurate
        if (width > height)
        {
          temp = height;
          height = width;
          width = temp;
        }
        
        //ratio is height divided by width
        ratio = height/width;

        if (color == red)
        {
          if (ratio <= triangle_ratio+tolerance && ratio >= triangle_ratio-tolerance)
          {
            Serial.println("Red Triangle");
          }
          
          else
          {
            Serial.println("Red Rectangle");
          }
        }
        else if (color == blue)
        {
          if (ratio <= triangle_ratio+tolerance && ratio >= triangle_ratio-tolerance)
          {
            Serial.println("Blue Triangle");
          }
          
          else
          {
            Serial.println("Blue Rectangle");
          }
        }
        else if (color == yellow)
        {
          if (ratio <= triangle_ratio+tolerance && ratio >= triangle_ratio-tolerance)
          {
            Serial.println("Yellow Triangle");
          }
          
          else
          {
            Serial.println("Yellow Rectangle");
          }
        }

      }
    }
  }  

}
