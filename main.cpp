#include <iostream>

using namespace std;
#include <math.h>

struct EulerAngles {
    double roll, pitch, yaw;
};

class Quaternion{
    private: 
        double w; //вещественная часть
        double x; //мнимые части
        double y;
        double z; // or double quat[4];
    
    public:
        // Quaternion(double qw, double qx, double qy, double qz):w(w), x(x), y(y), z(z) {}
        Quaternion(){
            w = 0;
            x = 0;
            y = 0;
            z = 0;
        }
        
        Quaternion(double value_w, double value_x, double value_y, double value_z){
            w = value_w;
            x = value_x;
            y = value_y;
            z = value_z;
        }
        
        void Print(){
            cout << w << " + " << x << "i +  "<< y<< "j + " << z << "k\n";
        }

        Quaternion conjugate(){ //сопряженный кватернион
            Quaternion new_q(w,-x,-y,-z);
            return new_q;
        }
        
        double magnitude(){  //модуль или длина кватерниона
            double norma = sqrt(w*w + x*x + y*y + z*z);
            return norma;
        }
        
        //void 
        Quaternion normalize(){ //нормализация кватерниона
            double norma = magnitude();
            Quaternion new_q(w/norma, x/norma, y/norma, z/norma);
            //new_q.Print();
            return new_q;
        }
        
        Quaternion inverse(){  //обратный кватернион
            Quaternion conjugate_q = Quaternion(w,x,y,z).conjugate();
            double norma = magnitude();
            double norma2 = pow(norma,2);
            Quaternion new_q(conjugate_q.w/norma2, conjugate_q.x/norma2, 
                             conjugate_q.y/norma2, conjugate_q.z/norma2);
            return new_q;
        }
        
        Quaternion operator+(const Quaternion & other) {
            Quaternion new_q(this->w + other.w, this->x + other.x, this->y + other.y, this->z + other.z);
            return new_q;
        }
        
        Quaternion operator-(const Quaternion & other) {
            Quaternion new_q(this->w - other.w, this->x - other.x, this->y - other.y, this->z - other.z);
            return new_q;
        }
        
        Quaternion operator*(const Quaternion & other) {
            Quaternion new_q;
            new_q.w = this->w * other.w - this->x * other.x - this->y * other.y - this->z * other.z;
            new_q.x = this->w * other.x + this->x * other.w + this->y * other.z - this->z * other.y;
            new_q.y = this->w * other.y - this->x * other.z + this->y * other.w + this->z * other.x;
            new_q.z = this->w * other.z + this->x * other.y - this->y * other.x + this->z * other.w;
            return new_q;
        }
        
        Quaternion operator*(const double& n){
            Quaternion new_q(n * this->w, n * this->x, n * this->y, n * this->z);
            return new_q;
        }
        
        Quaternion operator/(const Quaternion & other) {
           Quaternion q = other;
           Quaternion inv_q = q.inverse();
           Quaternion new_q;
           new_q.w = this->w * inv_q.w - this->x * inv_q.x - this->y * inv_q.y - this->z * inv_q.z;
           new_q.x = this->w * inv_q.x + this->x * inv_q.w + this->y * inv_q.z - this->z * inv_q.y;
           new_q.y = this->w * inv_q.y - this->x * inv_q.z + this->y * inv_q.w + this->z * inv_q.x;
           new_q.z = this->w * inv_q.z + this->x * inv_q.y - this->y * inv_q.x + this->z * inv_q.w;
           return new_q;
        }
        
        EulerAngles ToEulerAngles() {
            EulerAngles angles;
            
            // roll (x-axis rotation)
            double sinr_cosp = 2 * (this->w * this->x + this->y * this->z);
            double cosr_cosp = 1 - 2 * (this->x * this->x + this->y * this->y);
            angles.roll = std::atan2(sinr_cosp, cosr_cosp);
        
            // pitch (y-axis rotation)
            double sinp = 2 * (this->w * this->y - this->z * this->x);
            if (std::abs(sinp) >= 1)
                angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
            else
                angles.pitch = std::asin(sinp);
        
            // yaw (z-axis rotation)
            double siny_cosp = 2 * (this->w * this->z + this->x * this->y);
            double cosy_cosp = 1 - 2 * (this->y * this->y + this->z * this->z);
            angles.yaw = std::atan2(siny_cosp, cosy_cosp);
        
            return angles;
        }
    };
    
    

int main(){
    
    Quaternion q1(1,2,3,4);
    cout << "q1 = ";
    q1.Print();
    
    cout << "conjugate: q1* = ";
    q1.conjugate().Print();
    
    cout << "norma: |q1| = " << q1.magnitude() << '\n';
    
    cout << "normalize: q1' = ";
    q1.normalize().Print();
    
    cout << "inverse: q1^{-1} = ";
    q1.inverse().Print();

    cout << "q1 = ";
    q1.Print();

    Quaternion q2(5,6,7,8);
    cout << "q2 = ";
    q2.Print();
    
    cout << "q1 + q2 = ";
    Quaternion q3 = q1 + q2; 
    // q3 = q1 + operator+(q2);
    q3.Print();
    
    cout << "q1 - q2 = ";
    Quaternion q4 = q1 - q2; 
    q4.Print();
    
    // -60 + 12i + 30j + 24k
    cout << "q1 * q2 = ";
    Quaternion q5 = q1 * q2; 
    q5.Print();
    
    cout << "q1 * q1{-1} = ";
    q5 = q1 * q1.inverse(); 
    q5.Print();
    
    cout << "q1 / q2 = q1 * q2{-1} = ";
    Quaternion q6 = q1 / q2; 
    q6.Print();
    
    
    // Реализовать интегрирование кинематического уравнения Пуассона в кватернионной форме и 
    // организовать вывод углов Эйлера, получаемых из кватернионов.
    
    //https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    
    // При интегрировании уравнения Пуассона, после каждого шага 
    // интегрирования осуществлять нормировку получаемого кватерниона
    
    // Начальный кватернион
    Quaternion start_q(0.866, 0, 0.5, 0);
    // Начальная угловая скорость
    Quaternion omega(0, -3e-05, 0 , 0);
    
    Quaternion A = start_q;
    
    int k = 10;
    for (int i = 1; i <= k; i++) {
        A = (A * 0.5) * omega;
        A = A.normalize();
    };
    
    cout << "ToEulerAngles for q1: \n";
    EulerAngles EulerAng = q1.ToEulerAngles();
    cout << "roll = " << EulerAng.roll << "\npitch = " << EulerAng.pitch << "\nyaw = " <<  EulerAng.yaw; 
    
    return 0;
}
