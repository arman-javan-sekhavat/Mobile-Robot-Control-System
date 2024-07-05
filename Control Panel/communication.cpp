#include <winsock.h>
#include <string>

extern SOCKET serverSock;
extern SOCKET clientSock;


int startServer(void) {

    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    // socket()
    serverSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);


    // bind()
    hostent* localHost = gethostbyname("");
    char* localIP = inet_ntoa(*(struct in_addr*)*localHost->h_addr_list);

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(3000);
    server_addr.sin_addr.s_addr = inet_addr(localIP);


    bind(serverSock, (SOCKADDR*)&server_addr, sizeof(server_addr));

    // listen()
    int call = listen(serverSock, SOMAXCONN);

    return call;
}



int stopServer(void) {
    int call_1 = closesocket(serverSock);
    int call_2 = WSACleanup();

    return (call_1 | call_2);
}