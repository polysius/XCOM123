#include <iostream>
#include <memory>
#include <unordered_map>
#include <string>
#include <functional>
#include <vector>
#include "Module_Interface.h"

using namespace std;

// TODO : WRITE OPEN GL SPECIFIC INTERFACES HERE

// Auto Registration

template<typename T>
class AutoRegister {
public:
	AutoRegister(const string& name) {
		ModuleRegistry::Instance().RegisterFactory<T>(name);
	}
};

int main() {
	return 0;
}