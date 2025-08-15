#include <iostream>
using namespace std;
int main(){
    int count = 5;
    double distance = 3.7;
    char grade = 'A';
    bool done = false;

    cout<<"Count:" << count << "\n";
    cout<<"Distance:" << distance << "\n";
    cout<<"Grade:" << grade << "\n";
    cout<<"Done?"<<done<<"(0 = false, 1 = true)\n";

    // show bool as true/false instead of 0/1
    cout <<boolalpha;
    done = true;
    cout << "Done now? " << done << "\n";

    float f = 3.1415926f;
    double d=3.141592653580793;
    cout <<"float: "<< f << "\n";
    cout <<"double: "<< d <<"\n";

    unsigned int u =10;
    u = -1; // forced wrap around
    cout <<"unsigned after -1 assignment: " << u << "\n";
    
    return 0;
}

