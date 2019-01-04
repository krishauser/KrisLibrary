#ifndef ANY_VALUE_H
#define ANY_VALUE_H

// polymorphic container class
// adapted from boost::any

#include <typeinfo>
#include <algorithm>
#include <string>
#include <sstream>
#include <utility>
#include <KrisLibrary/errors.h>

//forward declarations for Clang on OSX
class AnyValue;
template<typename ValueType>
ValueType* AnyCast(AnyValue * operand);
template<typename ValueType>
const ValueType* AnyCast(const AnyValue * operand);

/**@brief A polymorphic container class that can contain data of any type.
 *
 * To retrieve the contained data, use AnyCast.
 */
class AnyValue
{
 public:
  // intializers
  AnyValue()
    : content(0)
    {}
  template<typename ValueType>
  AnyValue(const ValueType & value)
    : content(new holder<ValueType>(value))
    {}
  AnyValue(const AnyValue & other)
    : content(other.content ? other.content->clone() : 0)
    {}
  AnyValue(AnyValue && other) noexcept
    : content(other.content)
    {
      other.content = NULL;
    }
  ~AnyValue() { delete content; }

  //operators
   AnyValue & swap(AnyValue & rhs) {
     std::swap(content, rhs.content);
     return *this;
   }
   template<typename ValueType>
   AnyValue & operator=(const ValueType & rhs) {
     AnyValue(rhs).swap(*this);
     return *this;
   }
   AnyValue & operator=(const AnyValue & rhs) {
     AnyValue(rhs).swap(*this);
     return *this;
   }
   bool empty() const { return !content; }
   const std::type_info & type() const {
     return content ? content->type() : typeid(void);
   }
   template <class T>
   bool hastype() const { 
	   return &type() == &typeid(T);
   }
   template <class ValueType>
   bool operator == (const ValueType &rhs) const {
     if(&type() != &typeid(ValueType)) return false;
     return *AnyCast<ValueType>(this) == rhs;
   }

   template <class ValueType>
   bool operator != (const ValueType &rhs) const {
     return !operator == (rhs);
   }

 private:
   struct placeholder
   {
     virtual ~placeholder() { }
     virtual const std::type_info & type() const = 0;
     virtual placeholder * clone() const = 0;
   };

   template<typename ValueType>
   struct holder : public placeholder
   {
     holder(const ValueType & value) : held(value) {}
     virtual const std::type_info & type() const { return typeid(ValueType); }
     virtual placeholder * clone() const { return new holder(held); }

     ValueType held;  
   };

   template<typename ValueType> friend ValueType * AnyCast(AnyValue *);
   template<typename ValueType> friend ValueType * AnyCast_Quick(AnyValue *);
   template<typename ValueType> friend ValueType * AnyCast_Raw(AnyValue *);

   placeholder * content;
};

/// Retreive the data within the operand, or NULL if not of the correct type
template<typename ValueType>
ValueType * AnyCast(AnyValue * operand)
{
  return operand && operand->type() == typeid(ValueType)
    ? &static_cast<AnyValue::holder<ValueType> *>(operand->content)->held
    : 0;
}

/// Retreive the data within the operand, or NULL if not of the correct type
template<typename ValueType>
const ValueType * AnyCast(const AnyValue * operand)
{
  return AnyCast<ValueType>(const_cast<AnyValue *>(operand));
}

/// Retreive the data within the operand, or NULL if the operand is NULL.
/// This differs from AnyCast in that it does not check for the correct type,
/// which slightly improves performance, at the cost of requiring the caller
/// to first verify type  The caller must be very sure that the item's type
/// is correct!
template<typename ValueType>
ValueType * AnyCast_Raw(AnyValue * operand)
{
  return operand 
    ? &static_cast<AnyValue::holder<ValueType> *>(operand->content)->held
    : 0;
}

/// Retreive the data within the operand, or NULL if the operand is NULL.
/// This differs from AnyCast in that it does not check for the correct type,
/// which slightly improves performance, at the cost of requiring the caller
/// to first verify type  The caller must be very sure that the item's type
/// is correct!
template<typename ValueType>
const ValueType * AnyCast_Raw(const AnyValue * operand)
{
  return AnyCast_Raw<ValueType>(const_cast<AnyValue *>(operand));
}


/// Retreive the data within the operand, or NULL if not of the correct type.
/// This differs from AnyCast in that it checks for the correct
/// typeid by reference rather than by value, which does not work across DLL
/// boundaries but is faster.
template<typename ValueType>
ValueType * AnyCast_Quick(AnyValue * operand)
{
  return operand && (&operand->type() == &typeid(ValueType))
    ? &static_cast<AnyValue::holder<ValueType> *>(operand->content)->held
    : 0;
}

/// Retreive the data within the operand, or NULL if not of the correct type.
/// This differs from AnyCast in that it checks for the correct
/// typeid by reference rather than by value, which does not work across DLL
/// boundaries but is faster.
template<typename ValueType>
const ValueType * AnyCast_Quick(const AnyValue * operand)
{
  return AnyCast_Quick<ValueType>(const_cast<AnyValue *>(operand));
}

/// Coerces the data within the given value to the desired value T.  Returns
/// true if the coercion was successful, and result contains the coerced value.
///
/// Built-ins exist for bool, char, unsigned char, int, unsigned int, float
/// and double.
template <class T>
bool CoerceCast(const AnyValue& value,T& result)
{
  const T* val = AnyCast<T>(&value);
  if(val) {
    result = *val;
    return true;
  }
  return false;
}

template <> bool CoerceCast<bool>(const AnyValue& value,bool& result);
template <> bool CoerceCast<char>(const AnyValue& value,char& result);
template <> bool CoerceCast<unsigned char>(const AnyValue& value,unsigned char& result);
template <> bool CoerceCast<int>(const AnyValue& value,int& result);
template <> bool CoerceCast<unsigned int>(const AnyValue& value,unsigned int& result);
template <> bool CoerceCast<float>(const AnyValue& value,float& result);
template <> bool CoerceCast<double>(const AnyValue& value,double& result);
template <class T> bool LexicalCast(const T& value,std::string& result);
template <> bool LexicalCast(const AnyValue& value,std::string& result);
template <class T> std::string LexicalCast(const T& value);
template <class T> bool LexicalCast(const std::string& value,T& result);
template <> bool LexicalCast(const std::string& value,AnyValue& result);


template <class T> bool LexicalCast(const T& value,std::string& result)
{
  std::stringstream ss;
  ss<<value;
  if(ss) {
    result = ss.str();
    return true;
  }
  return false;
}

template <class T> std::string LexicalCast(const T& value)
{
  std::string result;
  if(!LexicalCast(value,result)) { return ""; }
  return result;
}

template <class T> bool LexicalCast(const std::string& value,T& result)
{
  std::stringstream ss(value);
  ss>>result;
  if(ss) return true;
  return false;
}


#endif
