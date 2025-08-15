#include <iostream>
using namespace std;

int main(){
    double a;
    double b;
    char op;
    char again = 'y';
    while (again == 'y' || again == 'Y'){
        cout << "Enter a number, an operand and a number ";
        cin >> a >> op >> b;
        double result = 0.0;
        bool ok = true;
        switch (op){
            case '+': result = a + b; break;
            case '-': result = a - b; break;
            case '*': result = a * b; break;
            case '/':
                if (b == 0) {
                    cout << "Division by zero\n";
                    ok = false;
                } else {
                    result = a / b;
                }
                break;
            default:
                cout << "Unknown operator\n";
                ok = false;
                break;
        }
          
        if (ok) {
            cout << "Result:" << result << "\n";
        }  

    cout << "Another? (y/n): ";
    cin >> again;
    }

    return 0;
}