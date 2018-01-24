/*

  g++ -o position-server -Wall -ansi -pedantic -O3 -pthread position-server.cpp -lboost_system -std=c++11

*/


#include <string>
#include <iostream>
#include <cstdlib>
#include <map>
#include <utility>

#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <boost/asio.hpp>

typedef std::pair<double,double>  Point;  // (x,y)

class SharedValue {

public:

  typedef std::map<std::chrono::system_clock::time_point,Point>   time_map;

private:

  time_map points;
  std::mutex lock;  

public:

  long int persist;
  
  SharedValue(void) : points(), lock(), persist(10) {}
  ~SharedValue(void) {}

  time_map operator()(void) {
    std::unique_lock<std::mutex> exclusion(lock);

    std::chrono::system_clock::time_point horizon = std::chrono::system_clock::now()-std::chrono::seconds(persist);
    time_map::iterator begin, up;
    begin = points.begin();
    if(begin != points.end() && (*begin).first < horizon) {
      up = points.upper_bound(horizon);
      points.erase(begin,up);
    }
    return points;
  }

  SharedValue& operator+=(const Point& d) {
    std::unique_lock<std::mutex> exclusion(lock);
    points[std::chrono::system_clock::now()] = d;
    return *this;
  }

  void clear(void) {
    std::unique_lock<std::mutex> exclusion(lock);
    points.clear();
  }

  void set_persistance(long int persistance) {
    std::unique_lock<std::mutex> exclusion(lock);
    persist = persistance;
  }
};

class ServiceThread {
private:

  typedef boost::asio::ip::tcp::iostream socket_stream;

  SharedValue&                      value;
  std::shared_ptr<socket_stream>  p_socket; // Sockets streams cannot be copied....

public:

  ServiceThread(SharedValue& val,
		boost::asio::ip::tcp::acceptor& acceptor) 
    : value(val), p_socket(new socket_stream()) {
    acceptor.accept(*(p_socket->rdbuf()));
  }

  // This is called internally at thread creation.
  ServiceThread(const ServiceThread& cp) 
    : value(cp.value), p_socket(cp.p_socket) {
  }

  ~ServiceThread(void) {
  }

  void operator()(void) {
    std::string op;
    double x,y;
    long int persist;
    socket_stream& socket = *p_socket;

    try {
      socket.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
      while(true) {
	socket >> op;
	if(op == "quit") 
	  break;
	if(op == "clear") 
	  value.clear();
	else if(op == "put") {
	  socket >> x >> y;
	  value += Point(x,y);
	}
	else if(op == "get") {
	  SharedValue::time_map::iterator iter,end;
	  SharedValue::time_map points = value();
	  socket << points.size() << std::endl;
	  for(iter=points.begin(), end=points.end(); iter != end; ++iter) {
	    Point& p = (*iter).second;
	    socket << p.first << ' ' << p.second << ' ' << std::endl;
	  }
	}
	else if(op == "persistance") {
	  socket >> persist;
	  value.set_persistance(persist);
	}
	else
	  std::cerr << "Operator '" << op << "' invalid" << std::endl;
      }
    }
    catch(std::exception& e) {
      std::cout << "Exception : " << e.what() << std::endl;
    }

    std::cout << "End of thread (client disconnects)" << std::endl;
  }
};


int main(int argc, char* argv[]) {
  if(argc!=3) {
    std::cerr << "Usage : " << argv[0] << " <port> <data persistance (seconds)>" << std::endl;
    return 1;
  }

  try {
    boost::asio::io_service        ios;
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), atoi(argv[1]));
    boost::asio::ip::tcp::acceptor acceptor(ios, endpoint); 
    SharedValue                    shared_value;

    shared_value.persist = atoi(argv[2]);
  
    while(true) {
      std::thread service(ServiceThread(shared_value,acceptor));
      service.detach();
    }
  }
  catch(std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
  
  return 0;
}
