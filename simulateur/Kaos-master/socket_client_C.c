/*
Prototype de socket développé pour le projet Perseus Avionique de l'Ecole Centrale de Lille
Prototype de client en C pour les utilisateurs du simulateur

A noter qu'il n'y a pas de prototype de serveur en C puisque le simulateur sera systématiquement serveur

Pour faire tourner et tester du C/C++ sur VSC, télécharger les extensions "C/C++" et "Code Runner"
Tuto ici : https://ludwiguer.medium.com/configure-visual-studio-code-to-compile-and-run-c-c-3cef24b4f690

NdA : en fonction du coeur Windows ou Unix, les codes sont différents - -'
Sockets créé à partir de : http://sdz.tdct.org/sdz/les-sockets.html
NdA : il est possible que le tuto ne marche pas sur Mac parce que les systèmes Unix vont 
avoir une différence de mort qui va nous casser les couilles. */

/* ////// INCLUSIONS /////// */

#if defined (WIN32)
    #include <winsock2.h>
    typedef int socklen_t;
#else
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <unistd.h>
    #define INVALID_SOCKET -1
    #define SOCKET_ERROR -1
    #define closesocket(s) close(s)
    typedef int SOCKET;
    typedef struct sockaddr_in SOCKADDR_IN;
    typedef struct sockaddr SOCKADDR;
#endif
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#define PORT 26000

/* ////// FONCTION UTILE /////// */

char** str_split(char* a_str, const char a_delim)
{
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;

    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }

    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);

    /* Add space for terminating null string so caller
       knows where the list of returned strings ends. */
    count++;

    result = malloc(sizeof(char*) * count);

    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);

        while (token)
        {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
        }
        assert(idx == count - 1);
        *(result + idx) = 0;
    }

    return result;
}

/* ////// CODES /////// */
 
int main(void)
{
    #if defined (WIN32)
        WSADATA WSAData;
        int erreur = WSAStartup(MAKEWORD(2,2), &WSAData);
    #else
        int erreur = 0;
    #endif
 
    SOCKET sock;
    SOCKADDR_IN sin;

    /* Paramètres de l'échange entre les programmes */
    char buffer[128] = "";
    char buffer_send[128] = "";
    double message_recu = 0;
    double message_to_send = 0;
    char** tokens;
 
    /* Si les sockets Windows fonctionnent */
    if(!erreur)
    {
        /* Création de la socket : AF_INET pour iternet ; AF_UNIX pour local */
        sock = socket(AF_INET, SOCK_STREAM, 0);
 
        /* Configuration de la connexion */
        sin.sin_addr.s_addr = htonl(INADDR_ANY); // inet_addr("127.0.0.1");
        sin.sin_family = AF_INET;
        sin.sin_port = htons(PORT);
 
        /* Si l'on a réussi à se connecter */
        if(connect(sock, (SOCKADDR*)&sin, sizeof(sin)) != SOCKET_ERROR)
        {
            printf("Connection à %s sur le port %d\n", inet_ntoa(sin.sin_addr), htons(sin.sin_port));
            
            /* Si l'on reçoit des informations : on les affiche à l'écran */
            while((recv(sock, buffer, 128, 0) != SOCKET_ERROR) & (message_to_send != 2000000))
            {
                ////////////////////////////////////////////////////////////////////////////
                ///////////////////////////// CODE DES CALCULS /////////////////////////////

                printf("On a reçu le brut %s.\n", buffer);
                tokens = str_split(buffer, ';');

                if (tokens)
                {
                    int i;
                    printf("On a reçu le message :\n");
                    for (i = 0; *(tokens + i); i++)
                    {
                        if (i==0) message_recu = atof(*(tokens + i));
                        printf("mesure %d = %s\n", i, *(tokens + i));
                        free(*(tokens + i));
                    }
                    printf("\n");
                    free(tokens);
                }

                //sleep(4);

                message_to_send = message_recu/2;

                snprintf(buffer_send, 50, "%f", message_to_send);
                printf("On renvoie le message %s.\n\n", buffer_send);
                //sleep(4);

                ////////////////////////////////////////////////////////////////////////////

                send(sock, buffer_send, sizeof(buffer_send), 0);
            } 
        }
        /* sinon, on affiche "Impossible de se connecter" */
        else
        {
            printf("Impossible de se connecter\n");
        }
 
        /* On ferme la socket */
        closesocket(sock);
 
        #if defined (WIN32)
            WSACleanup();
        #endif
    }
 
    /* On attend que l'utilisateur tape sur une touche, puis on ferme */
    getchar();
 
    return EXIT_SUCCESS;
}