import processing.serial.*;
import java.io.*;
int mySwitch=0;
int counter=0;
String [] subtext;
Serial myPort;

String receivedString;
int linefeed=10;
int oCtr=1;
int numTraces=1000000;
int startTraces=1;
int iCtr=1;
int frames=1000;
int new_fc = 1;
int tmp = 0;
int polled= 0;
int file_cnt=numTraces;
int cnt=1;
int repeat_cnt=1;
int dvfs_interval=1;
int dvfs_cnt=1;
int oCtr2;
int repeat_mode=0;
int poll_max=20000;
String [] subtext_arr=new String[dvfs_interval];

File dir = new File("../MATLAB/data/Ch3");

PrintWriter output;

void setup() {
  mySwitch=1;
  myPort = new Serial(this, "COM3", 115200);
  myPort.bufferUntil(linefeed);
  output = createWriter("../Output/output.txt");

  sendData("../Vectors/input.txt");
}

void draw() {
  if (mySwitch>0) {
    mySwitch=0;
  }

  serialEvent(myPort);
  if (receivedString=="null" || receivedString.isEmpty() || receivedString.length()==4) {
    receivedString="";
    //delay(0);
    mySwitch=0;
    myPort.clear();
  } else {
  }
} 


/* The following function will read from a CSV or TXT file */
void sendData(String myFileName) {
  File file=new File(myFileName);
  String[] pt_lines = loadStrings(file);
  //BufferedReader br=null;
  new_fc = countfiles(dir);
  tmp=new_fc;
  //try {
  //br=new BufferedReader(new FileReader(file));
  String text=null;
  //for (dvfs_cnt=1; dvfs_cnt<=dvfs_interval; dvfs_cnt++) {
  while (oCtr*frames<=numTraces+1) {
    do {
      long startTime=System.nanoTime();
      //text=br.readLine();
      String[] text_o=subset(pt_lines, (oCtr-1)*frames, frames);
      for (iCtr=0; iCtr<frames; iCtr++) {
        text=text_o[iCtr];
        text=text.replaceAll(" ", "");
        String[] key_pt=split(text, ',');
        text=key_pt[1];
        subtext_arr[dvfs_cnt-1]=text;
        subtext=text.split("");
        if (oCtr>=startTraces) {
          while (counter<32) {
            myPort.write(subtext[counter]);
            counter++;
          }
          myPort.write('\n');
          myPort.clear();
          mySwitch=0;
          counter=0;
        }
        println(iCtr);
        if (iCtr==frames-1) {
          println(text);
          println(iCtr);
        }
        print(receivedString);
        receivedString="";
        delay(1000);
      }
      polled = 0;
      while (polled<=poll_max) {
        delay(10);
        BufferedReader reader;
        String cfile_line; 
        reader=createReader("../MATLAB/data/count.txt");
        try {
          cfile_line=reader.readLine();
          if (cfile_line==null) {
            noLoop();
          } else {
            String[] pieces=split(cfile_line, TAB);
            tmp=int(pieces[0]);
          }
          polled=polled+1;
        } 
        catch (IOException e) {
          e.printStackTrace();
          cfile_line=null;
          polled=polled+1;
        }
        if (tmp>new_fc) {
          new_fc=tmp;
          println(polled);
          print(oCtr); 
          print(","); 
          println(new_fc);
          dvfs_cnt++;
          oCtr++;
          if (new_fc==file_cnt) {
            delay(10000);
          }
          output.print(receivedString);
          output.flush();
          receivedString="";
          long endTime=System.nanoTime(); 
          long ElapsedTimeMs=(endTime-startTime)/1000000; 
          println("ElapsedTimeMs "+ElapsedTimeMs+" ms.");
          delay(2000);
          break;
        } else if (polled==poll_max) {
          println("polled expired");
        }
      }
    } while (polled>poll_max);
    if (dvfs_cnt==dvfs_interval+1) {
      dvfs_cnt=1; 
      repeat_mode=1;
    }
  }
  output.close();
}

void serialEvent(Serial Port) {
  receivedString+=Port.readStringUntil(linefeed);
}

void keyPressed() {
  output.flush(); // Writes the remaining data to the file
  output.close(); // Finishes the file
  exit(); // Stops the program
}

int countfiles(File dir) {
  int k = 0;
  String[] children = dir.list();
  if (children == null) {
    return(-1);
  } else {
    for (int i=0; i<children.length; i++) {
      k++;
    }
    //println(k);
    return(k);
  }
}   