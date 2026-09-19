#pragma once

#include <pond/pond.h>
#include <unordered_set>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

namespace pond
{
    #define _CPP_PARAM_REF(p_ptr, extra_ptr) (*(c_type*extra_ptr)((uint8_t*)(p_ptr) + val_offset))

    template<typename T, typename c_type>
    class _ParameterBase
    {
    friend class _UntypedParameterBase;
    public:

        std::optional<T> getStrict(const std::unordered_set<T>& allowed_values = {}, bool log = true)
        {
            pond_parameter* p = api->get_parameter(api->ctx, (uint8_t*)name->c_str());
            if (p != NULL)
            {
                if (p->type == pt)
                {
                    T value{_CPP_PARAM_REF(p, )};
                    
                    if (auto it = allowed_values.find(value); it != allowed_values.end() || allowed_values.size() == 0)
                    {
                        free(p);
                        return value;
                    }
                    else if (log)
                    {
                        std::string list; list.reserve(1000);
                        for (auto& v : allowed_values)
                        {
                            list.append(", ");
                            if constexpr (std::is_same_v<T, std::string>) { list.append("'");list.append(v);list.append("'"); }
                            else if constexpr (std::is_same_v<T, bool>) list.append(v ? "true" : "false");
                            else list.append(std::to_string(v));
                        }

                        std::string string_value;
                        if constexpr (std::is_same_v<T, std::string>) string_value = "'" + value + "'";
                        else if constexpr (std::is_same_v<T, bool>) string_value = (value ? "true" : "false");
                        else string_value = std::to_string(value);

                        api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s' (%s) value must be in { %s }", name->c_str(), string_value.c_str(), &list.c_str()[2]);
                    }   
                }
                else if (log) api->log(
                    api->ctx, 
                    (uint8_t*)"[Error] Parameter '%s'.type (%s) != %s", 
                    name->c_str(), 
                    pond_parameter_type_to_string(p->type),
                    pond_parameter_type_to_string(pt)
                );
                free(p);
            }
            else if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s' not set", name->c_str());
            
            return std::nullopt;
        }

        void set(const T& val)
        {
            pond_parameter* p;
            if constexpr (std::is_same_v<T, std::string>)
            {
                p = (pond_parameter*)malloc(sizeof(pond_parameter) + val.size() + 1);
                p->value.String = (uint8_t*)p + sizeof(pond_parameter);
                memcpy(p->value.String, val.c_str(), val.size() + 1);
            }
            else
            {
                p = (pond_parameter*)malloc(sizeof(pond_parameter));
                _CPP_PARAM_REF(p, ) = val;
            }
            
            p->type = pt;
            api->set_parameter(api->ctx, (uint8_t*)name->c_str(), p);
        }

        T get(const T& default_val, const std::unordered_set<T>& allowed_values = {}, bool set_default = true)
        {
            if (auto ret = getStrict(allowed_values, false)) return std::move(ret.value());
            else
            {
                if (set_default) set(default_val);
                return default_val;
            }
        }
    private:
        _ParameterBase(std::string* _name, pond_api* _api, size_t _val_offset, pond_parameter_type _pt)
            : api(_api), name(_name), val_offset(_val_offset), pt(_pt) {}
        pond_api* api;
        std::string* name;
        size_t val_offset;
        pond_parameter_type pt;
    };

    template<typename T, typename c_type>
    class _ParameterArrayBase
    {
    friend class _UntypedParameterBase;
    public:

        std::optional<std::vector<T>> getStrict(uint32_t min_length = 0, uint32_t max_length = 0, bool log = true)
        {
            pond_parameter* p = api->get_parameter(api->ctx, (uint8_t*)name->c_str());
            if (p != NULL)
            {
                if (p->type == pt)
                {
                    if (check_array_length(p->array_length, min_length, max_length, log))
                    {
                        std::vector<T> vec(p->array_length);
                        for (int i = 0; i < vec.size(); i++) vec[i] = T{_CPP_PARAM_REF(p, *)[i]};
                        free(p);
                        return vec;
                    }
                }
                else if (log) api->log(
                    api->ctx, 
                    (uint8_t*)"[Error] Parameter '%s'.type (%s) != %s", 
                    name->c_str(), 
                    pond_parameter_type_to_string(p->type),
                    pond_parameter_type_to_string(pt)
                );
                free(p);
            }
            else if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s' not set", name->c_str());
        
            return std::nullopt;
        }

        void set(const std::vector<T>& val)
        {
            pond_parameter* p;
            if constexpr (std::is_same_v<T, std::string>)
            {
                size_t total_size = 0;
                for (auto& s : val) total_size += s.size() + 1;

                p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(uint8_t*) * val.size() + total_size);
                p->value.StringArray = (uint8_t**)((uint8_t*)p + sizeof(pond_parameter));

                for (uint32_t i = 0, offset = 0; i < val.size(); i++)
                {
                    p->value.StringArray[i] = (uint8_t*)((uint8_t*)p + sizeof(pond_parameter) + sizeof(uint8_t*) * val.size() + offset);
                    memcpy(p->value.StringArray[i], val[i].c_str(), val[i].size()+1);
                    offset += val[i].size()+1;
                }
            }
            else
            {
                p = (pond_parameter*)malloc(sizeof(pond_parameter) + sizeof(c_type) * val.size());
                _CPP_PARAM_REF(p, *) = (c_type*)((uint8_t*)p + sizeof(pond_parameter));
                for (uint32_t i = 0; i < val.size(); i++) _CPP_PARAM_REF(p, *)[i] = val[i];
            }
            
            p->type = pt;
            p->array_length = val.size();
            api->set_parameter(api->ctx, (uint8_t*)name->c_str(), p);
        }

        std::vector<T> get(const std::vector<T>& default_val, uint32_t min_length = 0, uint32_t max_length = 0, bool set_default = true)
        {
            if (auto ret = getStrict(min_length, max_length, false)) return std::move(ret.value());
            else
            {
                if (set_default) set(default_val);
                return default_val;
            }
        }

    private:
        bool check_array_length(uint32_t length, uint32_t min, uint32_t max, bool log)
        {
            if (min == max && min != 0 && length != min)
            {
                if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length != %d", name->c_str(), min);
                return false;
            }
            if (length < min)
            {
                if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length < %d", name->c_str(), min);
                return false;
            }
            if (length > max && max != 0)
            {
                if (log) api->log(api->ctx, (uint8_t*)"[Error] Parameter '%s'.length > %d", name->c_str(), max);
                return false;
            }
            return true;
        }

        _ParameterArrayBase(std::string* _name, pond_api* _api, size_t _val_offset, pond_parameter_type _pt)
            : api(_api), name(_name), val_offset(_val_offset), pt(_pt) {}
        pond_api* api;
        std::string* name;
        size_t val_offset;
        pond_parameter_type pt;
    };

    class _UntypedParameterBase
    {
    friend class ModuleBase;
    public:

    _ParameterBase<int32_t, int32_t> asInt();
    _ParameterArrayBase<int32_t, int32_t> asIntArray();
    _ParameterBase<double, double> asDouble();
    _ParameterArrayBase<double, double> asDoubleArray();
    _ParameterBase<bool, bool> asBool();
    _ParameterArrayBase<bool, bool> asBoolArray();
    _ParameterBase<std ::string, char*> asString();
    _ParameterArrayBase<std ::string, char*> asStringArray();

    private:

        _UntypedParameterBase(std::string* _name, pond_api* _api) : api(_api), name(_name) {}
        pond_api* api;
        std::string* name;
    };

}