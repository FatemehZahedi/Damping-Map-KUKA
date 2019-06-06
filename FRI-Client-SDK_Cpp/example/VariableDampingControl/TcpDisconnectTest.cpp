#include <stdio.h>
#include <iostream>

#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <thread>
#include <chrono>

#include <signal.h>


/* Global Variables */
const std::string IPADDR("192.168.0.102");
const int PORT = 50041;

/* Keyboard Commands */
enum class KeyboardCommand{
    CONNECT = 1,
    SENDMSG = 2,
    QUIT = 100
};

/* Function Prototypes */
KeyboardCommand GetKeyboardCommand();
void ConnectEmgDataStream(int & sock, struct sockaddr_in & sinRemote);
void SendMsg(int & sock);
void sigpipe_handler(int s){
    printf("Caught SIGPIPE\n");
}

int main(int argc, char** argv){
    /* Setup sigpipe handler */
    signal(SIGPIPE, sigpipe_handler);

    /* Initialize Socket Parameters */
    in_addr_t ipAddress = inet_addr(IPADDR.c_str());
    struct sockaddr_in sinRemote;
    sinRemote.sin_family = AF_INET;
    sinRemote.sin_addr.s_addr = ipAddress;
    sinRemote.sin_port = htons(PORT);
    int sock = socket(AF_INET, SOCK_STREAM, 0);

    /* Get commands and execute */
    KeyboardCommand keycmd;
    while (true){
        keycmd = GetKeyboardCommand();
        if (keycmd == KeyboardCommand::CONNECT){
            ConnectEmgDataStream(sock, sinRemote);
        }
        else if (keycmd == KeyboardCommand::SENDMSG){
            SendMsg(sock);
        }
        else if (keycmd == KeyboardCommand::QUIT){
            exit(1);
        }
    }


}

/* Function Definitions */
void ConnectEmgDataStream(int & sock, struct sockaddr_in & sinRemote){
    /* Set socket to non-blocking, then connect.  Use getsockopt to validate connection */
    fcntl(sock, F_SETFL, O_NONBLOCK);
    int connval = connect(sock, (struct sockaddr*)&sinRemote, sizeof(struct sockaddr_in));

    printf("connect return val: %d\n", connval);
    // int error = 0;
    // socklen_t len = sizeof (error);
    // int retval = getsockopt (sock, SOL_SOCKET, SO_ERROR, &error, &len);
    //
    // /* Check error codes */
    // if (retval != 0) {
    //     /* there was a problem getting the error code */
    //     fprintf(stderr, "error getting socket error code: %s\n", strerror(retval));
    //     return;
    // }
    //
    // if (error != 0) {
    //     /* socket has a non zero error status */
    //     fprintf(stderr, "socket error: %s\n", strerror(error));
    // }

    // /* Reset socket to blocking mode */
    // int flags = fcntl(sock, F_GETFL, 0);
    // flags &= ~O_NONBLOCK;
    // fcntl(sock, F_SETFL, flags);
}

void SendMsg(int & sock){
    char strTemp[5] = " ";
    int retval = send(sock, strTemp, strlen(strTemp), 0);
    printf("send return val: %d\n", retval);
}


KeyboardCommand GetKeyboardCommand(){
    char keyboard_input[256] = "";
    bool validInput = false;
    int keyboard_val;
    KeyboardCommand retval;
    while (!validInput){
        printf("Input Command Number\n");
        printf("\t1\t- Connect\n");
        printf("\t2\t- Send Msg\n");
        printf("\t100\t- Quit\n");
        // reset string to null
        memset(keyboard_input, 0, sizeof(keyboard_input));
        // get keyboard input
        fgets(keyboard_input, 256, stdin);
        // convert to integer
        keyboard_val = atoi(keyboard_input);

        // compare with enum vals
        if (keyboard_val == (int) KeyboardCommand::CONNECT){
            retval = KeyboardCommand::CONNECT;
            validInput = true;
        }
        else if (keyboard_val == (int) KeyboardCommand::SENDMSG){
            retval = KeyboardCommand::SENDMSG;
            validInput = true;
        }
        else if (keyboard_val == (int) KeyboardCommand::QUIT){
            retval = KeyboardCommand::QUIT;
            validInput = true;
        }
        else{
            printf("Invalid Input.  Your Input: %s\n", keyboard_input);
        }
    }
    return retval;
}
