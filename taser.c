/*
 * taser2.c
 * This file is part of taser 2.0
 *
 * Copyright (C) 2011 - Octavio Benedí Sánchez
 *
 * taser 2.0 is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * taser 2.0 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <getopt.h>
#include <sys/stat.h>
#include <signal.h>

#define FALSE 0
#define TRUE 1

#define STDIN 0
#define BUFFER 1024

static int serialPort = -1;
static int DEBUG = 0;
static int CR = 0;
static int speed = B115200;
static int exit_signal = 0;
typedef struct
{
    char *name;
    int flag;
} speed_spec;
struct termios previous_term_options,term_options;
static speed_spec speeds[] =
{
    {"1200", B1200},
    {"2400", B2400},
    {"4800", B4800},
    {"9600", B9600},
    {"19200", B19200},
    {"38400", B38400},
    {"57600", B57600},
    {"115200", B115200},
    {NULL, 0}
};

static int csize=0;
unsigned char cBuff[BUFFER+1];

int file_exist (char *filename)
{
    struct stat   buffer;
    return (stat (filename, &buffer) == 0);
}

//------------max()--------------------------//
//Returns the largest value of the two inputs//
//-------------------------------------------//
int max(int a, int b)
{
    return (a > b ? a : b);
}


static void * CtrlCSignalRelease( void * temp_pt )
{
    sleep(2);
    exit_signal = 0;
    if (DEBUG)fprintf(stderr,"\nCTRL+C trap released\n");
}


void sigint_handler(int s)
{
    if (exit_signal)
    {
        tcsetattr(serialPort, TCSANOW, &previous_term_options);
        fprintf(stderr,"Goodbye!\n");
        exit(0);
    }
    (void) signal(SIGINT, sigint_handler);
    if (DEBUG) fprintf(stderr,"\nPress CTRL+C another time to exit within next two seconds!\n");
    exit_signal=1;

    char ctrlc = 0x03;
    write(serialPort,&ctrlc,1);

    pthread_t CtrlCSignalReleaseThread;
    if ( pthread_create( &CtrlCSignalReleaseThread, NULL, CtrlCSignalRelease, NULL ) != 0 )
    {
        if (DEBUG) fprintf(stderr,"\nCannot create control+c release thread, next Ctrl+c will exit\n");
    }
}



int main(int argc, char *argv[])
{
    fd_set rset;
    int c;
    while (1)
    {
        int option_index = 0;
        char *short_options = "p:s";
        static struct option long_options[] =
        {
            /* These options set a flag. */
            {"verbose", no_argument, &DEBUG, 1},
            {"cr", no_argument, &CR, 1},
            /* These options don't set a flag.
            We distinguish them by their indices. */
            {"help",  no_argument, 0, 'h'},
            {"speed",  required_argument, 0, 's'},
            {0, 0, 0, 0}
        };

        c = getopt_long_only (argc, argv, short_options,
                              long_options, &option_index);
        /* Detect the end of the options. */
        if (c == -1)
            break;

        switch (c)
        {
        case 0:
            break;

        case 'h':
            fprintf(stderr,"Usage: %s [OPTIONS] Port\n",argv[0]);
            fprintf(stderr,"Options:\n");
            fprintf(stderr,"\t--help: display this help and exit\n");
            fprintf(stderr,"\t--verbose: set verbose mode\n");
            fprintf(stderr,"\t--cr: replace \\n with \\r\\n\n");
            fprintf(stderr,"\t--speed [value]: set value as serial port speed\n");
            fprintf(stderr,"\t\tAllowed speed values are: 1200 2400 4800 9600 19200 38400 57600 115200\n");
            fprintf(stderr,"Author: Octavio Benedi Sanchez\n");
            exit(0);
            break;

        case 's':
        {
            speed_spec *s;
            int speed_ok = 0;
            for (s = speeds; s->name; s++)
            {
                if (strcmp(s->name, optarg) == 0)
                {
                    speed = s->flag;
                    speed_ok = 1;
                    break;
                }
            }
            if (speed_ok)
            {
                printf("Selected speed `%s'\n", optarg);
            }
            else
            {
                printf("No valid speed value `%s' provided\nDefault speed(115200) used instead.\n", optarg);
            }
            break;
        }

        default:
            fprintf(stderr,"try: %s --help\n",argv[0]);
            exit(0);
            abort ();
        }
    }
    if (DEBUG) printf("verbose mode on\n");
    if (DEBUG) if (CR) printf("CR mode on\n");

    /* Print any remaining command line arguments (not options). */
    if (optind < argc)
    {
        int selected_port = FALSE;
        while (optind < argc)
        {
            if (file_exist(argv[optind]))
            {
                if (DEBUG) printf ("Using %s as port\n", argv[optind]);
                selected_port = TRUE;
                break;
            }
            else
            {
                if (DEBUG) printf ("%s is available\n", argv[optind]);
                optind++;
            }
        }
        if (!selected_port)
        {
            printf ("Serial port not available.\n");
            exit(EXIT_FAILURE);
        }
    }


    if ((serialPort = open(argv[optind], O_RDWR | O_NOCTTY | O_NDELAY)) == -1)
    {
        fprintf(stderr,"Unable to open the serial port %s - \n", argv[optind]);
        exit(EXIT_FAILURE);
    }
    else
    {
        //fcntl(serialPort, F_SETFL, O_NONBLOCK); /* set up non-blocking read */
        fcntl(serialPort, F_SETFL, 0);
    }
    tcgetattr(serialPort, &previous_term_options);
    tcgetattr(serialPort, &term_options);
    cfsetispeed(&term_options, speed);
    cfsetospeed(&term_options, speed);
    term_options.c_cflag |= (CLOCAL | CREAD);
    /*No parity*/
    term_options.c_cflag &= ~PARENB;
    term_options.c_cflag &= ~CSTOPB;
    term_options.c_cflag &= ~CSIZE;
    term_options.c_cflag |= CS8;
    /*raw input:
     * making the applycation ready to receive*/
    term_options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    /*Ignore parity errors*/
    term_options.c_iflag |= ~(INPCK | ISTRIP | PARMRK);
    term_options.c_iflag |= IGNPAR;
    term_options.c_iflag &= ~(IXON | IXOFF | IXANY | IGNCR | IGNBRK);
    term_options.c_iflag |= BRKINT;
    /*raw output
     * making the applycation ready to transmit*/
    term_options.c_oflag &= ~OPOST;
    /*aply*/
    tcsetattr(serialPort, TCSANOW, &term_options);

    (void) signal(SIGINT, sigint_handler);

    FD_ZERO(&rset); //Clear file descriptors in the rset set
    while (1)
    {
        FD_SET(STDIN,&rset);//Set STDIN
        FD_SET(serialPort,&rset);//Set serialPort
        select(max(STDIN,serialPort)+1,&rset,NULL,NULL,NULL);
        if (FD_ISSET(STDIN,&rset))
        {
            //Is there stuff to read from the standard input
            if ((csize = read(STDIN, cBuff, BUFFER)) >= 1)
            {
                if (CR)
                {
                    for (c = 0 ; c < csize; c++)
                    {
                        if (cBuff[c] == '\n' )
                        {
                            write(serialPort,"\r",1);
                        }
                        write(serialPort,&cBuff[c],1);
                    }
                }
                else
                {
                    write(serialPort, &cBuff, csize);
                }
            }
        }
        if (FD_ISSET(serialPort,&rset))
        {
            //Is there stuff to read from the standard input
            if ((csize = read(serialPort, cBuff, BUFFER)) >= 1)
            {
                write(STDIN, &cBuff, csize);
            }
        }
    }
    tcsetattr(serialPort, TCSANOW, &previous_term_options);
    exit (0);
}

