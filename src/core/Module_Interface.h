#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include "core/EngineContext.h"

// Note: `using namespace std;` was removed from this header. A using-directive in a header
// leaks into every file that includes it, which can cause surprising name clashes.

// Blueprint for the modules
class IModule {
public:
	// virtual : Means this function can be overwritten by a child class
	virtual ~IModule() = default;
	// Modules are created by name through a factory, so they cannot take constructor arguments.
	// The context hands them the shared services they need instead (window, settings).
	virtual bool Initialize(EngineContext& context) = 0;
	virtual void Shutdown() = 0;
	virtual const char* GetName() const = 0;
	virtual const char* GetVersion() const = 0;
};

// Specific module interfaces (such as IRenderModule) live next to the systems they describe,
// for example render/IRenderModule.h.

class ModuleRegistry {
	// modules : Stores currently running modules objects
	// factories : stores instructions for creating new modules
private:
	// Hash table <key, value (smart pointer which points to the running module )>
	// Eg : "Vulkan" -> [Pointer to an actual VulkanRenderer object that's running]
	std::unordered_map<std::string, std::unique_ptr<IModule>> modules;
	// Eg : "Vulkan" -> [A function that when called, creates a new VulkanRenderer]
	std::unordered_map<std::string, std::function<std::unique_ptr<IModule>()>> factories; // unique_ptr : smart pointer

	ModuleRegistry() = default;
public:

	// Implementing Singleton Pattern to ensure there is only one Module Registry
	// When Instance() is called for the first time, 'instance' object is created
	// Every subsequent call results in the same instance object being called.
	static ModuleRegistry& Instance() {
		static ModuleRegistry instance;
		return instance;
	}

	// Register a factory function for creating modules
	// Template<typename T> means "This function works with any type, call it T for now"
	template<typename T>
	void RegisterFactory(const std::string& name) {
	// [] = Capture clause (empty = captures nothing from surrounding scope)
	// () = Parameters (empty = takes no parameters)
	// { return std::make_unique<T>(); } function body : stores a function which creates a new object of type T and wraps it in a smart pointer.
	// Eg : When you call RegisterFactory<VulkanRenderer>("Vulkan"), the compiler generates:
	//	factories[name] = []() {return std::make_unique<VulkanRenderer>(); };
		factories[name] = []() { return std::make_unique<T>(); };
	}

	bool CreateModule(const std::string& name, EngineContext& context) {
		// looks for a factory function associated with 'name'
		// Returns an iterator (pointer-like object) to the map entry
		auto it = factories.find(name);
		if (it == factories.end()) {
			std::cerr << "Factory Not found for module : " << name << std::endl;
			return false;
		}

		// This creates a new module object
		// it->second is the value (the factory function)
		auto module = it->second();
		if (!module->Initialize(context)) {
			std::cerr << "Failed to initialize module: " << name << std::endl;
			// Let the module release whatever it managed to create before failing.
			module->Shutdown();
			return false;
		}

		modules[name] = std::move(module);
		return true;
	}

	// Get Module
	IModule* GetModule(const std::string& name) {
		auto it = modules.find(name);
		// condition ? value_if_true : value_if_false
		return (it != modules.end()) ? it->second.get() : nullptr;
	}

	// Get a module cast to specific interface
	// Dynamic cast to convert a pointer from one type to another
	// Eg : Converting IModule* to IRenderModule*
	template<typename T>
	T* GetModule(const std::string& name) {
		IModule* module = GetModule(name);
		return module ? dynamic_cast<T*>(module) : nullptr;
	}

	// shutdown all modules
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

// Auto Registration
// Writing `static AutoRegister<MyModule> registration("MyModule");` in a module's .cpp file
// registers its factory before main() runs, so main() never needs to know the concrete class.
// (Moved here from Module_Interface.cpp so other files can use it.)
template<typename T>
class AutoRegister {
public:
	explicit AutoRegister(const std::string& name) {
		ModuleRegistry::Instance().RegisterFactory<T>(name);
	}
};
