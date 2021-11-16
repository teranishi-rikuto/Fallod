#include <Camera.h>
#include <SDHCI.h>
SDClass theSD;


int intPin = 4;
int gCounter = 0;


void setup()
{
  
  pinMode(LED0, OUTPUT);
  pinMode(intPin, INPUT_PULLUP);
  Serial.begin(115200);
  while (!Serial);
  printf("a\n");
  theCamera.begin();
  printf("c\n");
  while (!theSD.begin());
  printf("b\n");
  

   theCamera.setStillPictureImageFormat(
    CAM_IMGSIZE_VGA_H, CAM_IMGSIZE_VGA_V, CAM_IMAGE_PIX_FMT_JPG
   );
  
  //  theCamera.setStillPictureImageFormat(
  //    CAM_IMGSIZE_QUADVGA_H, CAM_IMGSIZE_QUADVGA_V, CAM_IMAGE_PIX_FMT_JPG
  //   );

}

void loop()
{
  

  CamImage img = theCamera.takePicture();

  char filename[16] = {0};
  if(img.isAvailable())
  {
    
    digitalWrite(LED0,HIGH);
    printf("gCounter = %d\n", gCounter);
    printf("camera\n");
    sprintf(filename,"PICT%02d.JPG", gCounter);
    
    if(theSD.exists(filename))
    {
      theSD.remove(filename);
      printf("delete\n");
    }
    File myFile = theSD.open(filename, FILE_WRITE);
    
    myFile.write(img.getImgBuff(), img.getImgSize());
    myFile.close();

    gCounter++;
    if(gCounter == 10)gCounter=0;
  }
  digitalWrite(LED0,LOW);

  delay(1000);

}
