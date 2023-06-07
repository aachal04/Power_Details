#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include "serial_main.h"

void power_details(float current, float voltage_pse, float voltage_pd)
{
   float ppse = voltage_pse * current;
   float ppd = voltage_pd * current;
   float cable_voltage_drop = voltage_pse - voltage_pd;
   float cable_power_loss = ppse - ppd;
   
   printf("Current(mA):%.5f\n",current);
   printf("PSE Voltage (VDC):%.5f\n",voltage_pse);
   printf("PD Voltage (VDC):%.5f\n",voltage_pd);
   printf("PSE power:%.5f\n",ppse);
   printf("PD power:%.5f\n",ppd);
   printf("Cable Voltage Drop:%.5f\n",cable_voltage_drop);
   printf("Cable Voltage Loss:%.5f\n",cable_power_loss);
}



int serial_read_write()
{
    int fd = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        perror("Failed to open UART port");
        return -1;
    }
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200); // Set the baud rate
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver and set local mode
    options.c_cflag &= ~PARENB;          // Disable parity bit
    options.c_cflag &= ~CSTOPB;          // Set one stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;             // Set data bit to 8
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHONL | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | INLCR | IGNCR | ICRNL);
    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~ONLCR;
    options.c_cc[VTIME] = 13;
    options.c_cc[VMIN] = 0;  

    tcsetattr(fd, TCSANOW, &options); // Apply the settings

    char message[7] = "ADC\r\n";
    int bytes_written = write(fd, message, strlen(message)); // Write the message to the port
    if (bytes_written < 0)
    {
        perror("Failed to write to UART port");
        return -1;
    }
    
    char buffer[2000];
    memset(buffer, '\0', sizeof(buffer));
    int bytes_read = 0;
    int i = 0;

    while (1)
    {
        bytes_read = read(fd, &buffer[i], 1);
        if (bytes_read < 0)
        {
            perror("Failed to read the UART port");
            return -1;
        }
        else if (bytes_read == 0)
        {
            // No more data available to read
            break;
        }

      
        i++;
    }
    printf("\nReceived Message \n%s\n", buffer);
    
    float current; 
    float voltage_pse;
    float voltage_pd;
    
    char* token = strtok(buffer,"\r\n");
    while(token != NULL)
    {
       char key[50];
       float value = 0.0;
       if(sscanf(token,"%[^:]:%f",key,&value) == 2)
       {
          if(strcmp(key,"Current")==0)
          {
             current = value;
          }
          else if(strcmp(key,"PSE Voltage")==0)
          {
             voltage_pse = value;
          }
          else if(strcmp(key,"PD Voltage")==0)
          {
             voltage_pd = value;
          }
       }
       token = strtok(NULL,"\r\n");
    }
    power_details(current, voltage_pse, voltage_pd);

    // printf("\nReceived Message \n%s\n", buffer);
    close(fd);
    return 0;
}


int main(int argc, char *argv[])
{  

    if (argc < 3)
    {
        printf("Usage: %s <arg1> <arg2>\n", argv[0]);
        return -1;
    }
    
    
    if(strcmp(argv[1], "power")==0 && strcmp(argv[2], "details")==0)
    {
       if(argc > 3)
       {
           printf("Invalid command!! more than two arguments\n");
       }
       else
       {
           serial_read_write();
       }
    }
    
    else if(argv[1]!="power" && argv[2]!="details")
    {
       printf("Enter the valid command!!!\n");
    }
  
    return 0;
    
}


