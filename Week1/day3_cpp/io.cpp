#include <iostream> 
using namespace std;

int main(){
    int age;
    cout << "Enter your age: ";
    cin >> age; //waits for you to type something, stores it in age
    cout << "In 5 years you will be "<< age +5 << " years old.\n";
    return 0;
}