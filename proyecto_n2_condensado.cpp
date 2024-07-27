#include <iostream>                                             // Libreria de entrada y salida
#include <iomanip>                                              // Libreria para incluir set Precision (Reducir decimales mostrados en pantalla)
#include <cmath>                                                // Libreria matematica
using namespace std;                                            // Espacio de nombres std
namespace Estanques {                                           // Se define la clase Tanque
    class Tanque {                                              
    protected: float volumen; int caudal;                       //Atributos protegidos para acceder desde clases derivadas
    public:                                                     //Metodos publicos que comparten las clases derivadas (Polimorfismo)
        virtual float CalcularVolumen() const = 0;              //Metodo para calcular el volumen
        virtual float CalcularNivel(float volumen) const = 0;   //Metodo para calcular el nivel a partir de un volumen dado
        virtual void SetterPrametros() = 0;                     //Metodo para la solicitud de los parametros
        float get_volumen() const { return volumen; }           //Metodo para la obtener el volumen
        void set_volumen(float vol) { volumen = vol; }          //Metodo para setear el volumen
        int get_caudal() const { return caudal; }               //Metodo para obtener el caudal
        void set_caudal(float caud) {                           //Metodo para aproximar el caudal
            caudal = (int)caud;
            if (caudal % 2 != 0) { caudal -= 1; }}
        virtual ~Tanque() {};};                                 //Destructor virtual
    class Prisma : public Tanque {
    private: float largo, ancho, alto;                          // Atributos privados de la clase Prisma derivada de Tanque
    public: 
        Prisma(float largo, float ancho, float alto) : largo(largo), ancho(ancho), alto(alto) {};   //Constructor
        float CalcularVolumen() const override {return largo * ancho * alto;}                       //Metodo para el calculo del volumen
        float CalcularNivel(float volumen) const override{return volumen/(largo*ancho);}            // Metodo obtencion altura
        void SetterPrametros() override {                                                           //Metodo para la peticion de los parametros
            cout << "\t- Largo [m]: "; cin >> largo; cout << "\t- Ancho [m]: "; cin >> ancho; cout << "\t- Alto [m]: ";  cin >> alto;
            volumen = CalcularVolumen();}
        float get_largo() const { return largo; }                                                   //Metodo para la obtencion del largo
        float get_ancho() const { return ancho; }                                                   //Metodo para la obtencion del ancho
        float get_alto() const { return alto; }};                                                   //Metodo para la obtencion del alto
    class Cilindro : public Tanque {
    private: float radio, alto;                                                                     //Atributos privados para la clase cilindro derivada de Tanque
    public: //Metodos
        Cilindro(float radio, float alto) : radio(radio), alto(alto) {};                            //Constructor
        float CalcularVolumen() const override {return M_PI * pow(radio, 2) * alto;}                //Metodo para el calculo del volumen  
        void SetterPrametros() override {                                                           //Metodo para la solicitud de parametros
            cout << "\t- Radio [m]: "; cin >> radio; cout << "\t- Alto [m]: ";  cin >> alto;
            volumen = CalcularVolumen();}
        float CalcularNivel(float volumen) const override{return volumen/(radio*radio);}            //Metodo obtencion altura
        float get_radio() const { return radio; }                                                   //Metodo obtencion radio
        float get_alto() const { return alto; }};                                                   //Metodo obtencion alto
    class PIDController {
    public:
        PIDController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), prevError_(0), integral_(0) {}     //Constructor
        float calculate(float setpoint, float current, float dt, int caudal) {                                      //Metodo para calcular la salida del controlador
            float error = setpoint - current;                                                                       //Calculo del error (referencia - valor_actual)
            float P = kp_ * error;                                                                                  //Parte proporcional, a mayor Kp, mayor es el error               
            integral_ += error * dt;                                                                                //Integral, error acumulado, area bajo la curva en un intervalo de tiempo 
            float I = ki_ * integral_;                                                                              //Parte integral aumenta con el valor de Ki
            float D = kd_ * ((error - prevError_) / dt);                                                            //Parte derivativa, diferencia entre valor de error anterior y actual en un intervalo de tiempo
            float output = P + I + D;                                                                               //Accion de control, ajusta el caudal dado que es la variable que se puede controlar
            prevError_ = error;                                                                                     //Representa el error anterior
            if (output > caudal) { output = caudal; } if (output < -caudal) { output = -caudal; }                   //Condicional BIAS
            return output;}                                                                                         //El metodo devuelve el valor de la acción de control
    private: float kp_, ki_, kd_, prevError_, integral_;};                                                          //Atributos privados de la clase
};// Cierre de espacios de nombre
void pausa() { //Funcion que permite la pausa del programa
    cout << "\nPresione Enter para continuar..." << endl; getchar(); while (getchar() != '\n');}
int main() { //Funcion principal
    using namespace Estanques; //Se agrega espacio de nombres
    while (true) { //Condicional de bucle
        Tanque* estanque = nullptr; // Se crea un puntero estanque que apunta hacia la clase Tanque, inicialmente como nulo 
        float caudal, Kp, Ki, Kd, porcentaje_vaciado, porcentaje_llenado; int forma; //Variables locales
        // Interaccion con el usuario para la seleccion de la forma
        cout << "Seleccione la forma del estanque: \n\n\t1: Cilindrico\n\t2: Prisma Rectangular\n\t0: Salir: \n\nIngrese un valor: "; cin >> forma; cout << endl;
        // Condicional para el instanciamento de la clase correcta de acuerdo a la forma seleccionada
        switch (forma) {
            case 0: return 0;
            case 1: estanque = new Cilindro(0, 0); break;
            case 2: estanque = new Prisma(0, 0, 0); break;
            default: cout << "\nValor ingresado erroneo\n\n" << endl; continue;}
		cout<<"Ingrese los parametros del estanque: \n"<<endl; //Peticion al usuario de los parametros del estanque
        estanque->SetterPrametros(); estanque->set_volumen(estanque->CalcularVolumen()); //Llamado al metodo para el calculo del volumen
        do {cout << "\t- Caudal [m3/s]: "; cin >> caudal; //Solicitud para ingreso de caudal
        } while (caudal < 2);
		estanque->set_caudal(caudal); //Llamado a metodo para el seteo del atributo caudal
		cout<<"\nIngrese los porcentajes de llenado y vaciado del estanque: \n"<<endl; //Peticion de los porcentajes de vaciado y llenado
        cout << "\t- Ingrese el porcentaje de llenado: "; cin >> porcentaje_llenado;
        cout << "\t- Ingrese el porcentaje de vaciado: "; cin >> porcentaje_vaciado;
		cout<<"\nIngrese los parametros del controlador PID: \n"<<endl; //Peticion de los parametros del controlador PID
        cout << "\t- Ingrese el valor de Kp: "; cin >> Kp;
        cout << "\t- Ingrese el valor de Ki: "; cin >> Ki;
        cout << "\t- Ingrese el valor de Kd: "; cin >> Kd;
        PIDController* pid = new PIDController(Kp, Ki, Kd); //Se instancia la clase PIDController
        float nivel_actual = 0,altura_actual = 0, dt = 0.0001, tiempo_total = 0, tiempo_max = 100000; int i = 0; // Inicializacion de variables para el control de volumen del estanque
        cout << fixed << setprecision(2); //Solo dos decimales en pantalla
        float volumen_llenado = estanque->get_volumen() * porcentaje_llenado / 100; float volumen_vaciado = estanque->get_volumen() * porcentaje_vaciado / 100;
        cout<<"\n---------------------- Proceso de llenado... ----------------------\n"<<endl;// Proceso de llenado
        while (nivel_actual <= volumen_llenado && tiempo_total<tiempo_max) {
            float ajuste_caudal = pid->calculate(volumen_llenado, nivel_actual, dt, estanque->get_caudal()); //Valor de salida del PID
            nivel_actual += ajuste_caudal* dt;altura_actual = estanque->CalcularNivel(nivel_actual);tiempo_total += dt; // Variables de interes                                                                  
            if (fmod(tiempo_total,1) <= dt){i+=1;cout<<"\t"<<i<<"\t"<<nivel_actual<<" [m3] "<<"\t"<<altura_actual<<" [m] "<<"\t"<<tiempo_total<<" [s]"<<endl;}; //Condicional para muestreo                                                                                 //Se calcula el tiempo necesario para alcanzar el estado estacionario
        };i++;cout<<"\t"<<i<<"\t"<<nivel_actual<<" [m3] "<<"\t"<<altura_actual<<" [m] "<<"\t"<<tiempo_total<<" [s]"<<endl;
        float tiempo_llenado = tiempo_total, volumen_llenado_PID = nivel_actual, altura_llenado = altura_actual; i = 0; nivel_actual = volumen_llenado; tiempo_total = 0;
        cout<<"\n-------------------- Proceso de vaciado... --------------------\n"<<endl;// Proceso de vaciado
        while (nivel_actual >= volumen_vaciado && tiempo_total<tiempo_max) {
            float ajuste_caudal = pid->calculate(volumen_vaciado, nivel_actual, dt, estanque->get_caudal()); //Valor de salida del PID
            nivel_actual -= ajuste_caudal*dt; altura_actual = estanque->CalcularNivel(nivel_actual); tiempo_total += dt; //Variables de interes
            if (fmod(tiempo_total,1) <= dt){i+=1;cout<<"\t"<<i<<"\t"<<nivel_actual<<" [m3] "<<"\t"<<altura_actual<<" [m] "<<"\t"<<tiempo_total<<" [s]"<<endl;};                                                                                  //Se calcula el tiempo necesario para alcanzar el estado estacionario
        };i++;cout<<"\t"<<i<<"\t"<<nivel_actual<<" [m3] "<<"\t"<<altura_actual<<" [m] "<<"\t"<<tiempo_total<<" [s]"<<endl; 
        if (tiempo_total >= tiempo_max) { cout << "\nNo se alcanzo el nivel de vaciado deseado en el tiempo maximo permitido...\n" << endl; continue;} //Condicional en caso de que el PID no pueda cumplir su objetivo
        float tiempo_vaciado = tiempo_total, volumen_vaciado_PID = nivel_actual, altura_vaciado = altura_actual;
        // Resultados
        cout<<"\n|---------------------------------------------------------------------------|\n Resultados del control del volumen del estanque mediante un controlador PID\n|---------------------------------------------------------------------------|\n"<<endl;
        cout<<"Parametros del controlador:\n\n\t- Kp: "<<Kp<<"\n\t- Ki: "<<Ki<<"\n\t- Kd: "<<Kd<<endl;
        if (dynamic_cast<Cilindro*>(estanque) != nullptr) { // Mostrar parámetros del cilindro
            Cilindro* cilindro = dynamic_cast<Cilindro*>(estanque);
            cout << "\nParametros del estanque (Cilindrico):\n\n"<< "\t- Radio: " << cilindro->get_radio() << " [m]\n"<< "\t- Alto: " << cilindro->get_alto() << " [m]\n"<<"\t- Volumen total: " << cilindro->get_volumen() << " [m3]\n" << "\t- Caudal llenado y vaciado: " << cilindro->get_caudal() << " [m3/s]" << endl;
        } else if (dynamic_cast<Prisma*>(estanque) != nullptr) { // Mostrar parámetros del prisma
            Prisma* prisma = dynamic_cast<Prisma*>(estanque);
            cout << "\nParametros del estanque (Prisma Rectangular):\n\n"<< "\t- Largo: " << prisma->get_largo() << " [m]\n"<< "\t- Ancho: " << prisma->get_ancho() << " [m]\n"<< "\t- Alto: " << prisma->get_alto() << " [m]\n"<< "\t- Volumen total: " << prisma->get_volumen() << " [m3]\n" << "\t- Caudal llenado y vaciado: " << prisma->get_caudal() << " [m3/s]" << endl;}
        cout << "\nProceso de llenado al " << porcentaje_llenado << "%:" << "\n\n\t- Volumen alcanzado: " << volumen_llenado_PID << " [m3]" << "\n\t- Volumen deseado: " << volumen_llenado << " [m3]"<< "\n\t- Altura deseada: " << estanque->CalcularNivel(estanque->CalcularVolumen()*porcentaje_llenado/100)<< " [m]" << "\n\t- Altura alcanzada: " << altura_llenado << " [m]"<< "\n\t- Tiempo para llenado: " << tiempo_llenado << " [s]" << endl;
        cout << "\nProceso de vaciado al " << porcentaje_vaciado << "%:" << "\n\n\t- Volumen alcanzado: " << volumen_vaciado_PID << " [m3]" << "\n\t- Volumen deseado: " << volumen_vaciado << " [m3]"<< "\n\t- Altura deseada: " << estanque->CalcularNivel(estanque->CalcularVolumen()*porcentaje_vaciado/100)<< " [m]" << "\n\t- Altura alcanzada: " << altura_vaciado << " [m]"<< "\n\t- Tiempo para vaciado: " << tiempo_vaciado << " [s]" << endl;
        delete estanque; delete pid; pausa();
    }return 0;}