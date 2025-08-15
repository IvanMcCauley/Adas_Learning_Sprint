#include <iostream>
using namespace std;

int main(){
    int arr[5];
    cout << "Enter 5 numbers:\n"; //PROMPT
    for (int i=0; i<5; ++i){ 
        cin >> arr[i];// ALLOWS USER TO TYPE IN 5 NUMBERS
    }
    int sum = 0; //INITIALIZES SUM
    int maxv = arr[0]; //INITIALIZES MAX AS INDEX 0
    for (int i = 0; i<5; ++i){
        sum += arr[i];   // ITERATES THROIGH THE ARRAY AND ADDS EACH NUMBER ONTO IT AS A RUNNING SUM
        if (arr[i] > maxv) maxv = arr[i];// ITERATES THROUGH THE ARRAY AND REPLACES MAXV WITH A NUMBER IN THE ARRAY IF ITS BIGGER THAN THE CURRENT MAXV
    }    
    cout << "Sum = " << sum << "\n";
    cout << "Max: " << maxv << "\n";
    
    return 0;

}    
