Coammander (Folder: Project)
//Compile mit:     g++ TCPEchoClient_Commander.cpp shared_memory.cpp -lpthread -o TCPEchoClient_Commander
//Start mit:       ./TCPEchoClient_Commander 127.0.0.1 10001
Listeners(Folder: Project)
//Compile with:  g++ TCPEchoClient_Lidar.cpp shared_memory.cpp -lpthread  -o TCPEchoClient_Lidar
//Start with: ./TCPEchoClient_Lidar 127.0.0.1 10000
//Compile with:  g++ TCPEchoClient_Odometrie.cpp shared_memory.cpp -lpthread  -o TCPEchoClient_Odometrie
//Start with: ./TCPEchoClient_Odometrie 127.0.0.1 10002

Server for Listeners (Folder: Listeners):
//Compile with: g++ TCPEchoServer.cpp -o TCPEchoServer
//Start with ./TCPEchoServer 10000
//Compile with: g++ TCPEchoServer_Odo.cpp -o TCPEchoServer_Odo
//Start with ./TCPEchoServer_Odo 10002
Server for Commander (Folder: Commander):
//Compile with: g++ TCPEchoServer.cpp -o TCPEchoServer
//Start with ./TCPEchoServer 10001


Startreihenfolge:
1-3) Alle drei Server
4) Coammander
5) Listnerer Lidar
6) Listener Odometrie