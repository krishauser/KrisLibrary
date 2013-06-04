#ifndef COMMAND_LINE_H
#define COMMAND_LINE_H

#include <vector>
#include <sstream>

namespace CommandLine {

enum BasicType { None,Bool,Int,Float,Double,String };

struct CommandArg
{
	CommandArg() : type(None),name(NULL),data(NULL) {}
	CommandArg(BasicType type_input,const char* name_input,void* data_input)
		:type(type_input),name(name_input),data(data_input) {}

	BasicType type;
	const char* name;
	void* data;
};

struct CommandBase
{
	CommandBase() :line(NULL) {}
	virtual ~CommandBase() {}
	void AddBoolArg(const char* name, bool& val);
	void AddIntArg(const char* name, int& val);
	void AddFloatArg(const char* name, float& val);
	void AddDoubleArg(const char* name, double& val);
	void AddStringArg(const char* name, const char*& val);
	const char* GetArgName(int arg) const { return args[arg].name; }

	virtual const char* Name() = 0;
	virtual void PrintDescription(std::ostream& out);
	virtual void PrintDetailedDescription(std::ostream& out);
	virtual const char* Description() { return "unknown function."; }
	virtual size_t NumArgs() { return args.size(); }
	virtual int MinInputs() { return 0; } 
	virtual int MaxInputs() { return 0; } //if < 0, possibly infinite }
	virtual int NumOutputs() { return 0; } //can only be 0 or 1, for now
	virtual bool ProcessLine();
	virtual bool SetArg(int arg, const char* val);
	virtual int Do() = 0; 	//return > 0 if success, <=0 on failure
	
	const char* GetInput(int i) const { return inputs[i]; }
	const char* GetOutput(int i=1) { return outputs[i]; }
	int NumInputs() const { return inputs.size(); }

	std::vector<const char*>* line;  //the input line is divided into args,inputs,outputs
	std::vector<const char*> inputs;
	std::vector<const char*> outputs;
	std::vector<CommandArg> args;
};



struct CommandAuto : public CommandBase
{
	CommandAuto(const char* name_input, const char* desc_input=NULL)
		:name(name_input),desc(desc_input) {}
  virtual ~CommandAuto() {}
	virtual const char* Name() { return name; }
	virtual const char* Description() { if(desc) return desc; else return CommandBase::Description(); }

	const char* name;
	const char* desc;
};

//T is assumed to have istream >> defined
template <class T>
struct CommandInputValue : public CommandAuto
{
	CommandInputValue(const char* name_input, T* target_input)
		:CommandAuto(name,": sets a value."),target(target_input) {}
	virtual size_t NumArgs() { return 1; }
	virtual bool SetArg(int arg, const char* val)
	{
		std::istringstream str;
		str.str(val);
		str >> *target;
		if(!str)
			return false;
		return true;
	}
	virtual int Do() { return 1; }

	T* target;
};

template <class T>
struct CommandSetValue : public CommandAuto
{
	CommandSetValue(const char* name_input, T* target_input, const T& val_input)
		:CommandAuto(name_input,": sets a value."),target(target_input),val(val_input) {}
	virtual int Do()
	{
		*target = val;
		return 1;
	}

	T* target;
	const T& val;
};

void PrintUsage(const char* appName);
void PrintCommands(CommandBase* commands [], int numCommands);
int ReadArguments(CommandBase* commands [], int numCommands, int argc, const char** argv);

} //namespace CommandLine
#endif
