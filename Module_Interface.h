#pragma once

#include <iostream>
#include <memory>
#include <unordered_map>
#include <string>
#include <functional>
#include <vector>

using namespace std;

// Blueprint for the modules
class IModule {
public:
	// virtual : Means this function can be overwritten by a child class
	virtual ~IModule() = default;
	virtual bool Initialize() = 0;
	virtual void Shutdown() = 0;
	virtual const char* GetName() const = 0;
	virtual const char* GetVersion() const = 0;
};

// Specific Module Interface

class IRenderModule : public IModule {
public:
	// TODO : Functions Definitions Related to the renderes will go here
	virtual void Render() = 0;
	virtual void SetResolution(int width, int height) = 0;
};

class ModuleRegistry {
	// modules : Stores currently runnign modules objects 
	// factories : stores instructions for creating new modules 
private:
	// Hash table <key, value (smart pointer which points to the running module )>
	// Eg : "OpenGL" -> [Pointer to an actual OpenGLRenderer object that's running]
	unordered_map<string, unique_ptr<IModule>> modules; 
	// Eg : "OpenGL" -> [A function that when called, creates a new OpenGLRenderer]
	unordered_map<string, function<unique_ptr<IModule>()>> factories; //unique_ptr : smart pointer
	
	ModuleRegistry() = default;
public:

	// Implementing Singleton Pattern to ensure there is only one Module Registery
	// When Instance() is called for the first time, 'instance' object is created
	// Every subsequent call results in the same instance object being called.
	static ModuleRegistry& Instance() {
		static ModuleRegistry instance;
		return instance;
	}

	// Register a factory function for creating modules 
	// Template<typename T> means "This function works with any type, call it T for now"
	template<typename T>
	void RegisterFactory(const string& name) {
	// [] = Capture clause (empty = captures nothing from surrounding scope)
	// () = Parameters (empty = takes no parameters)
	// { return std::make_unique<T>(); } function body : stores a function which creates a new object of type T and wraps it in a smart pointer.
	// Eg : When you call RegisterFactory<OpenGLRenderer>("OpenGL"), the compiler generates:
	//	factories[name] = []() {return make_unique<OpenGLRenderer>(); };
		factories[name] = []() {return make_unique<T>(); };
	}

	bool CreateModule(const string& name) {
		// looks for a factory function assosicated with 'name'
		// Returns an iterator (pointer-like object) to the map entry
		auto it = factories.find(name); 
		if (it == factories.end()) {
			cerr << "Factory Not found for module : " << name << endl;
			return false;
		}

		// This creates a new module object
		// it->second is the value (the factory function)
		auto module = it->second(); 
		if (!module->Initialize()) {
			std::cerr << "Failed to initialize module: " << name << std::endl;
			return false;
		}

		modules[name] = move(module);
		return true;
	}

	// Get Module
	IModule* GetModule(const string& name) {
		auto it = modules.find(name);
		// condition ? value_if_true : value_if_false
		return (it != modules.end()) ? it->second.get() : nullptr;
	}

	// Get a module cast to specific interface
	// Dynamic cast to convert a pointer from one type to another
	// Eg : Converting IModule* to IRenderModule*
	template<typename T>
	T* GetModule(const string& name) {
		IModule* module = GetModule(name);
		return module ? dynamic_cast<T*>(module) : nullptr;
	}

	//shutdown all modules 
	void ShutdownAll() {
		for (auto& [name, module] : modules) {
			module->Shutdown();
		}
		modules.clear();
	}

	// List all loaded modules
	void ListModules() const {
		std::cout << "Loaded modules:\n";
		for (const auto& [name, module] : modules) {
			std::cout << "  - " << module->GetName()
				<< " (v" << module->GetVersion() << ")\n";
		}
	}
};