#include <iostream>

int main() {
    int x = 37;               // obična varijabla 'x'
    int* p = &x;              // pointer 'p' koji pokazuje na x

    std::cout << "Vrijednost x: " << x << std::endl;
    std::cout << "Adresa x (&x): " << &x << std::endl;

    std::cout << "\nVrijednost pointera p (adresa na koju pokazuje): " << p << std::endl;
    std::cout << "Vrijednost na koju pokazuje p (*p): " << *p << std::endl;

    std::cout << "\nPromjena x preko pointera (*p = 100):" << std::endl;
    *p = 100;
    std::cout << "Nova vrijednost x: " << x << std::endl;
    std::cout << "Nova vrijednost *p: " << *p << std::endl;

    return 0;
}


