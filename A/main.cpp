
#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
using namespace std;

const int n=60; // tamaño horizontal del mapa
const int m=60; // tamaño diagonal del mapa

static int map[n][m]; //tamaño vertical del mapa
static int closed_nodes_map[n][m]; // mapa de nodos cerrados (provado)
static int open_nodes_map[n][m]; // mapa de nodos abiertos (sin verificar)
static int dir_map[n][m]; // mapa de direcciones
const int dir=8; // número de direciones posibles paar ir a cualquier posición
// if dir==4
//static int dx[dir]={1, 0, -1, 0};
//static int dy[dir]={0, 1, 0, -1};
// if dir==8
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

class node
{
    // posición actual
    int xPos;
    int yPos;
    // distancia total recorrida para alcanzar el final
    int level;
    // prioridad = nivel + distancia restante estimada
    int priority;  // pequeña mayor priridad

public:
    node(int xp, int yp, int d, int p)
    {xPos=xp; yPos=yp; level=d; priority=p;}

    int getxPos() const {return xPos;}
    int getyPos() const {return yPos;}
    int getLevel() const {return level;}
    int getPriority() const {return priority;}

    void updatePriority(const int & xDest, const int & yDest)
    {
        priority=level+estimate(xDest, yDest)*10; //A*
    }

    // mayor priridad en ir estrecho que diagonal
    void nextLevel(const int & i) // i: dirección
    {
        level+=(dir==8?(i%2==0?10:14):10);
    }

    // Estima distancia restante para el objetivo
    const int & estimate(const int & xDest, const int & yDest) const
    {
        static int xd, yd, d;
        xd=xDest-xPos;
        yd=yDest-yPos;

        // distancia Euclidian
        d=static_cast<int>(sqrt(xd*xd+yd*yd));

        // distancia Manhattan
        //d=abs(xd)+abs(yd);

        // distancia Chebyshev
        //d=max(abs(xd), abs(yd));

        return(d);
    }
};

// Determina priridad en la cola
bool operator<(const node & a, const node & b)
{
    return a.getPriority() > b.getPriority();
}

// Algoritmo estralla.
// Devuelve una cadena de números enteros.
string pathFind( const int & xStart, const int & yStart,
                 const int & xFinish, const int & yFinish )
{
    static priority_queue<node> pq[2]; // lista de nodos abiertos(aún no probada)
    static int pqi; // pq index
    static node* n0;
    static node* m0;
    static int i, j, x, y, xdx, ydy;
    static char c;
    pqi=0;

    // restablecer los mapas de nodos
    for(y=0;y<m;y++)
    {
        for(x=0;x<n;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // crear el nodo de inicio y lo inserta en la lista de nodos abiertos
    n0=new node(xStart, yStart, 0, 0);
    n0->updatePriority(xFinish, yFinish);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority();

    // Busqueda A*
    while(!pq[pqi].empty())
    {
        // obtener el nodo actual w (de mayor prioridad)
        // de la lista de nodos abierta
        n0=new node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x=n0->getxPos(); y=n0->getyPos();

        pq[pqi].pop(); // remueve el nodo de la lista abierta
        open_nodes_map[x][y]=0;
        // marca en el mapa los nodos cerrados
        closed_nodes_map[x][y]=1;

        // salir de la busqueda cuando se alcanza el estado del objeto
        //if((*n0).estimate(xFinish, yFinish) == 0)
        if(x==xFinish && y==yFinish)
        {
            // genera la ruta desde el final hasta el comienzo
            string path="";
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                path=c+path;
                x+=dx[j];
                y+=dy[j];
            }

            // colección de basura
            delete n0;
            // elimina nodos sobrantes
            while(!pq[pqi].empty()) pq[pqi].pop();
            return path;
        }

        // genera los movimientos en todas las dirreciones posibles de los nodos secundarios
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>n-1 || ydy<0 || ydy>m-1 || map[xdx][ydy]==1
                 || closed_nodes_map[xdx][ydy]==1))
            {
                // genera un hijo
                m0=new node( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xFinish, yFinish);

                // se agrega en la lista abierta de no estar
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // marca la dirección padre principal
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // actualiza la información
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // actualiza la información de dirección principal
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // remplaza el nodo
                    // vacía el pq del otro
                    // excepto el nodo que se va areemplazar
                    while(!(pq[pqi].top().getxPos()==xdx &&
                            pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // elimina el nodo deseado

                    // vacia el tamaño más grande pq al más pequeño
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // agrega el major nodo en su lugar
                }
                else delete m0; // elimina basura
            }
        }
        delete n0; // elimina basura
    }
    return ""; // no encontró ruta
}

int main()
{
    srand(time(NULL));

    // crea un mapa vacío
    for(int y=0;y<m;y++)
    {
        for(int x=0;x<n;x++) map[x][y]=0;
    }

    // llena la matriz con un patron "+"
    for(int x=n/8;x<n*7/8;x++)
    {
        map[x][m/2]=1;
    }
    for(int y=m/8;y<m*7/8;y++)
    {
        map[n/2][y]=1;
    }

    // selecciona aleatoreamente las ubicaciones de inicio y fanal
    int xA, yA, xB, yB;
    switch(rand()%8)
    {
        case 0: xA=0;yA=0;xB=n-1;yB=m-1; break;
        case 1: xA=0;yA=m-1;xB=n-1;yB=0; break;
        case 2: xA=n/2-1;yA=m/2-1;xB=n/2+1;yB=m/2+1; break;
        case 3: xA=n/2-1;yA=m/2+1;xB=n/2+1;yB=m/2-1; break;
        case 4: xA=n/2-1;yA=0;xB=n/2+1;yB=m-1; break;
        case 5: xA=n/2+1;yA=m-1;xB=n/2-1;yB=0; break;
        case 6: xA=0;yA=m/2-1;xB=n-1;yB=m/2+1; break;
        case 7: xA=n-1;yA=m/2+1;xB=0;yB=m/2-1; break;
    }

    cout<<"Map Size (X,Y): "<<n<<","<<m<<endl;
    cout<<"Start: "<<xA<<","<<yA<<endl;
    cout<<"Finish: "<<xB<<","<<yB<<endl;
    // get the route
    clock_t start = clock();
    string route=pathFind(xA, yA, xB, yB);
    if(route=="") cout<<"An empty route generated!"<<endl;
    clock_t end = clock();
    double time_elapsed = double(end - start);
    cout<<"Time to calculate the route (ms): "<<time_elapsed<<endl;
    cout<<"Route:"<<endl;
    cout<<route<<endl<<endl;

    // sigue la ruta del mapa
    if(route.length()>0)
    {
        int j; char c;
        int x=xA;
        int y=yA;
        map[x][y]=2;
        for(int i=0;i<route.length();i++)
        {
            c =route.at(i);
            j=atoi(&c);
            x=x+dx[j];
            y=y+dy[j];
            map[x][y]=3;
        }
        map[x][y]=4;

        // mostrar
        for(int y=0;y<m;y++)
        {
            for(int x=0;x<n;x++)
                if(map[x][y]==0)
                    cout<<".";
                else if(map[x][y]==1)
                    cout<<"O"; //obstacle
                else if(map[x][y]==2)
                    cout<<"S"; //start
                else if(map[x][y]==3)
                    cout<<"R"; //route
                else if(map[x][y]==4)
                    cout<<"F"; //finish
            cout<<endl;
        }
    }
    getchar();
   return(0);
}