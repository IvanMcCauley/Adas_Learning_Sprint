#include <iostream>
using namespace std;

int main(){
    int age;
    while (true){ 
        cout << "Enter your age: ";
        cin >> age;

        if (cin.fail()){
            cin.clear();
            cin.ignore(1000, '\n');
            cout << "Invalid input, please enter a number.\n";
        } else {
            break;

        }
    }
     cout << "You entered: " << age << ".\n";
     return 0;
}