import processing.video.*;

// the following parameters must fit the c++ code:
int totalNrOfUpdates=1999;

MovieMaker mm;

int[] x=new int[4];
int[] y=new int[4];
int[] o=new int[4];

int[][] area=new int[16][16];

color[] c=new color[4];

int currentStep=-1;

String data;
String[] L;
PFont f;

int frame=0;//1000000;

boolean showLegend = true;


void drawArea()
{
  if((data!=null)&&(currentStep>0))
  {
    // remove this to make a trace of agent movement
     background(255, 255, 255);

     if (showLegend && currentStep < 200)
     {
       fill(0, 0, 0);
       text("Black - pathfinder", 10, 40);
      
       fill(0, 200, 0);
       text("Green - food", 10, 65);
     }
     
     if (showLegend && currentStep > 150)
     {
       showLegend = false;
     }
     
     for(int t=0;t<1;t++)
     {
      String[] P=splitTokens(L[currentStep+t],"=");
      float minX=0.0,minY=0.0,maxX=0.0,maxY=0.0;
      float[] x=new float[P.length];
      float[] y=new float[P.length];
      float[] a=new float[P.length];
      float[] s=new float[P.length];
      int[] R=new int[P.length];
      int[] G=new int[P.length];
      int[] B=new int[P.length];
      for(int j=0;j<P.length;j++)
      {
          String[] D=splitTokens(P[j],",");
          x[j]=Float.parseFloat(D[0]);
          y[j]=Float.parseFloat(D[1]);
          a[j]=Float.parseFloat(D[2]);
          s[j]=Float.parseFloat(D[3]);
          R[j]=Integer.parseInt(D[4]);
          G[j]=Integer.parseInt(D[5]);
          B[j]=Integer.parseInt(D[6]);
      }
      
      for(int j=0;j<P.length;j++)
      {
            if(t==-1)
            {
              strokeWeight(2.0);
              stroke(color(0,0,0));
              line((int)(x[j]+(width/2)),(int)(y[j]+(height/2)),(int)(x[j]+(width/2)+(cos(a[j]*(PI/180.0))*3)),(int)(y[j]+(height/2)+(sin(a[j]*(PI/180.0))*3)));
              set((int)(x[j]+(width/2)),(int)(y[j]+(height/2)),color(32,32,32));
            }
            else
            {
              strokeWeight(1);
              fill(color(R[j],G[j],B[j]));
                    
              if(R[j] == 0 && G[j] == 0 && B[j] == 0)
              {
                triangle((int)(x[j]+(width/2)+(cos(a[j]*(PI/180.0))*s[j])),
                          (int)(y[j]+(height/2)+(sin(a[j]*(PI/180.0))*s[j])),
                          (int)(x[j]+(width/2)+(cos((a[j]-135)*(PI/180.0))*s[j])),
                          (int)(y[j]+(height/2)+(sin((a[j]-135)*(PI/180.0))*s[j])),
                          (int)(x[j]+(width/2)+(cos((a[j]+135)*(PI/180.0))*s[j])),
                          (int)(y[j]+(height/2)+(sin((a[j]+135)*(PI/180.0))*s[j])));
              }
              else
              {
                ellipse(x[j]+(width/2), y[j]+(height/2), s[j], s[j]);
              }
              
              // un-comment to project predator retina
              /*if(R[j] == 255 && G[j] == 0 && B[j] == 0)
              {
                strokeWeight(2.0);
                fill(0);
                int predatorVisionDist = 200;
                line((int)(x[j]+(width/2)),(int)(y[j]+(height/2)),(int)(x[j]+(width/2)+(cos((a[j]+90)*(PI/180.0))*predatorVisionDist)),(int)(y[j]+(height/2)+(sin((a[j]+90)*(PI/180.0))*predatorVisionDist)));
                line((int)(x[j]+(width/2)),(int)(y[j]+(height/2)),(int)(x[j]+(width/2)+(cos((a[j]-90)*(PI/180.0))*predatorVisionDist)),(int)(y[j]+(height/2)+(sin((a[j]-90)*(PI/180.0))*predatorVisionDist)));
                curve((int)(x[j]+(width/2)+(cos((a[j]-180)*(PI/180.0))*predatorVisionDist*8)), (int)(y[j]+(height/2)+(sin((a[j]-180)*(PI/180.0))*predatorVisionDist*8)),
                      (int)(x[j]+(width/2)+(cos((a[j]-90)*(PI/180.0))*predatorVisionDist)), (int)(y[j]+(height/2)+(sin((a[j]-90)*(PI/180.0))*predatorVisionDist)),
                      (int)(x[j]+(width/2)+(cos((a[j]+90)*(PI/180.0))*predatorVisionDist)), (int)(y[j]+(height/2)+(sin((a[j]+90)*(PI/180.0))*predatorVisionDist)),
                      (int)(x[j]+(width/2)+(cos((a[j]+180)*(PI/180.0))*predatorVisionDist*8)), (int)(y[j]+(height/2)+(sin((a[j]+180)*(PI/180.0))*predatorVisionDist*8)));
              }*/
            }
        }
     }
    if((currentStep&1)==0)
    {
       frame++;
       //saveFrame("frame-"+frame+".png");
       mm.addFrame();
       
       /*if (frame == 56000)
       {
         mm.finish();
         exit();
       }*/
    } 
  }
}

void updateArea(){
  /*
  int[] nx=new int[4];
  int[] ny=new int[4];
  int[] no=new int[4];
  int i,j;
  for(i=0;i<4;i++){
    nx[i]=(int)data.charAt((currentStep*12)+(i*3))-(int)'A';
    ny[i]=(int)data.charAt((currentStep*12)+(i*3)+1)-(int)'A';
    no[i]=(int)data.charAt((currentStep*12)+(i*3)+2)-(int)'A';
  }
  for(i=0;i<4;i++){
    if(((nx[i]!=x[i])||(ny[i]!=y[i]))&&(area[nx[i]][ny[i]]==2)){
      area[nx[i]+(nx[i]-x[i])][ny[i]+(ny[i]-y[i])]=2;
    }
    area[x[i]][y[i]]=0;
    x[i]=nx[i];
    y[i]=ny[i];
    area[x[i]][y[i]]=1;
    o[i]=no[i];
  }
  */
}

void setup()
{
  size(806, 806);
  f = createFont("Calibri", 20, false);
  textFont(f);
  textAlign(LEFT);
  
  int s = second();
  int mi = minute();
  int h = hour();
  int d = day();
  int mo = month();
  int y = year();
  
  String movname = "video-" + y + "-" + mo + "-" + d + "-" + h + "-" + mi + "-" + s + ".mov";
  
  mm = new MovieMaker(this, width, height, movname, 30, MovieMaker.ANIMATION, MovieMaker.BEST);
  background(255, 255, 255);
  fill(0);
  noStroke();
  c[0]=color(0,0,0);
  c[1]=color(255,100,100);
  c[2]=color(0,255,0);
  c[3]=color(255,255,255);
  drawArea();
}

void keyReleased(){
  currentStep=0;
}
 
void draw()
{
  if(currentStep==-1){
    BufferedReader in;
    PrintStream out;
    Socket s;
    
    try{
      String host = "127.0.0.1";   // change

      //text( "Connecting to " + host + " ...", 10, 20 );
      s = new Socket( host, 2002 );
 
      //text( "Connection established to " + s.getInetAddress(), 10, 30 );
 
      in = new BufferedReader( new InputStreamReader( s.getInputStream() ) );
      //out = new PrintStream( s.getOutputStream() );
      data=in.readLine();
      L=splitTokens(data,"N");
//      print(data);
      in.close();
      //out.close();
      currentStep=0;
    }
    catch ( Exception e )
    {
      //println( "Error" + e );
    }
  }
  else{
    //background(#000000);
    drawArea();
    updateArea();
    currentStep++;
    if(data!=null)
    if(currentStep>=totalNrOfUpdates)
    {
      currentStep=-1;
      
      if(data.indexOf("X") != -1)
      {
        mm.finish();
        exit();
      }
    }
    //frame++;
    //saveFrame("frame-"+frame+".png"); 
  }
} 

void keyPressed() {
  if (key == ' ')
  {
     mm.finish();  // Finish the movie if space bar is pressed!
     exit();
  }
}
