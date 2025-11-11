# include <iostream>
using namespace std;

int main(){
    int i = 1;
    for (int i =1; i <=10; ++i) {
        cout << i << " ";
    }
    cout << "\n";

    while (i <= 10) {
        cout << i << " ";
        ++i;
    }

    cout << "\n";

    int k =1;
    do {
        cout << k << " "; 
        ++k;
     } while (k <= 10);
   
    cout << "\n";

    cout << "Type an integer: ";
    int n;
    cin >> n;
    
    long long fact = 1;
    for (int t =2; t<=n; ++t){
        fact *= t;
    }
    cout << n << "! = " << fact << "\n";

    long long sum = 0;
    for (int t = 1; t <=n; ++t) {
        sum +=t;
    }
    cout << "Sum 1.." << n << " = " << sum << "\n";
    return 0;
}
   