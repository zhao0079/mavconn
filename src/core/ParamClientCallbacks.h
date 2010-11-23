#ifndef _ParamClientCallbacks_H__
#define _ParamClientCallbacks_H__

#include <string>

class PCCallback
{
    public:
        PCCallback(void* userData = 0) : userData_(userData) {}

        virtual void operator()(const std::string& name, float value) = 0;

    protected:
        void* userData_;
};

template <typename T>
class PCCallbackNameValue : public PCCallback
{
    public:
        PCCallbackNameValue(void (*callback) (const std::string&, T, void*), void* userData = 0) : PCCallback(userData), callback_(callback) {}

        inline void operator()(const std::string& name, float value)
            { (*this->callback_)(name, (T)value, this->userData_); }

    private:
        void (*callback_) (const std::string&, T, void*);
};

template <typename T>
class PCCallbackValue : public PCCallback
{
    public:
        PCCallbackValue(void (*callback) (T, void*), void* userData = 0) : PCCallback(userData), callback_(callback) {}

        inline void operator()(const std::string&, float value)
            { (*this->callback_)((T)value, this->userData_); }

    private:
        void (*callback_) (T, void*);
};

template <class C, typename T>
class PCCallbackClassNameValue : public PCCallback
{
    public:
        PCCallbackClassNameValue(void (C::*callback) (const std::string&, T), C* object) : PCCallback(object), callback_(callback) {}

        inline void operator()(const std::string& name, float value)
            { (((C*)this->userData_)->*this->callback_)(name, (T)value); }

    private:
        void (C::*callback_) (const std::string&, T);
};

template <class C, typename T>
class PCCallbackClassValue : public PCCallback
{
    public:
        PCCallbackClassValue(void (C::*callback) (T), C* object) : PCCallback(object), callback_(callback) {}

        inline void operator()(const std::string&, float value)
            { (((C*)this->userData_)->*this->callback_)((T)value); }

    private:
        void (C::*callback_) (T);
};

template <typename T>
class PCCallbackVariable : public PCCallback
{
    public:
        PCCallbackVariable(T* variable) : variable_(variable) {}

        inline void operator()(const std::string&,float value)
            { *this->variable_ = (T)value; }

    private:
        T* variable_;
};

template <typename T>
inline PCCallback* createPCCallback(void (*callback) (const std::string&, T, void*), void* userData = 0)
    { return new PCCallbackNameValue<T>(callback, userData); }

template <typename T>
inline PCCallback* createPCCallback(void (*callback) (T, void*), void* userData = 0)
    { return new PCCallbackValue<T>(callback, userData); }

template <class C, typename T>
inline PCCallback* createPCCallback(void (C::*callback) (const std::string&, T), C* object)
    { return new PCCallbackClassNameValue<C, T>(callback, object); }

template <class C, typename T>
inline PCCallback* createPCCallback(void (C::*callback) (T), C* object)
    { return new PCCallbackClassValue<C, T>(callback, object); }

template <typename T>
inline PCCallback* createPCCallback(T* variable)
    { return new PCCallbackVariable<T>(variable); }

#endif /* _ParamClientCallbacks_H__ */
