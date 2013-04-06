/* 
 * File:   Complex.h
 * Author: lubos
 *
 * Created on March 27, 2013, 5:18 PM
 */

#ifndef COMPLEX_H
#define	COMPLEX_H

template <class T> struct Complex;
template <class T> std::ostream& operator<< (std::ostream& out, const Complex<T>& cpx);

template <class T>
struct Complex
{
    T real;

    T imag;
    
    Complex() 
    :real(0), imag(0) {}
    
    Complex(double real, double imag)
    : real(real), imag(imag) {}
    
    ~Complex() {}
  
    Complex operator+(const Complex& other)
    {        
        return Complex(this->real + other.real, this->imag + other.imag);
    }
    
    Complex operator-(const Complex& other)
    {        
        return Complex(this->real - other.real, this->imag - other.imag);
    }
    
    Complex operator*(const Complex& other)
    {        
        return Complex(real * other.real - imag * other.imag, real * other.imag + imag * other.real);
    }    
    
    Complex& operator+=(const Complex& other)
    {
        this->real += other.real;
        this->imag += other.imag;
        return (*this);
    }
    
    Complex& operator-=(const Complex& other)
    {
        this->real -= other.real;
        this->imag -= other.imag;
        return (*this);
    }

    Complex& operator*=(const Complex& other)
    {
        this->real = real * other.real - imag * other.imag;
        this->imag = real * other.imag + imag * other.real;
        return (*this);
    }
    
    Complex& operator/=(const double scalar)
    {
        this->real /= scalar;
        this->imag /= scalar;
        return (*this);
    }
    
    Complex operator*(const double scalar)
    {
        double re = this->real * scalar;
        double im = this->imag * scalar;
        return Complex(re, im);
    }
    
    Complex conjugate() 
    {
        return Complex(real, -imag); 
    }
    
    friend std::ostream& operator<< (std::ostream& out, const Complex& cpx)
    {
        out << "(" << cpx.real << " " << cpx.imag << "i)";
        return out;
    }
    
    float magnitude()
    {
        return (float) std::sqrt(real * real + imag * imag);
    }

    float phase()
    {
        return (float) std::atan2(imag, real);
    }
};

#endif	/* COMPLEX_H */

