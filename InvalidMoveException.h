#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <exception>
using namespace std;

class InvalidMoveException : public exception {
private:
  string msg;

public:
  InvalidMoveException(const string& msg) : msg(msg) {}
  virtual const char* what() const throw() { return msg.c_str(); }
  ~InvalidMoveException() throw() {}
};

#endif
