#pragma once

#include <pond/pond.h>

#include <string>
#include <vector>
#include <array>
#include <typeinfo>
#include <memory>
#include <functional>

namespace pond
{
    class ModuleBase;
    class Distributor;
    class Receiver;

    class ChannelsInfo
    {
    friend class Distributor;
    friend class Receiver;
    public:
        ChannelsInfo() {infos.reserve(16);info_strings.reserve(16);}

        template<typename T>
        ChannelsInfo& channel(const std::string& name)
        {
            if constexpr (std::is_same_v<T, void>) info_strings.push_back({name, ""});
            else info_strings.push_back({name, std::string(typeid(T).name()) + "_" + std::to_string(typeid(T).hash_code())});
            
            infos.push_back({.channel = (uint8_t*)info_strings[infos.size()][0].c_str(), .type = (uint8_t*)info_strings[infos.size()][1].c_str()});

            return *this;
        }

        template<typename T> using argtype_string = const std::string&;

        template<typename... Args>
        ChannelsInfo& channels(argtype_string<Args>... names)
        {
            ((channel<Args>(names)), ...);

            return *this;
        }
    private:
        std::vector<std::array<std::string, 2>> info_strings;
        std::vector<pond_dds_slot_info> infos;
    };

    class Distributor
    {
    friend class ModuleBase;
    public:
        Distributor() = default;
        
        void destroy()
        {
            if (id == -1) return;
            api->destroy_distributor(api->ctx, id);
        }

        template<typename T>
        void distribute_raw(T** data_ptrs)
        {
            if (id == -1) return;
            else api->distribute(api->ctx, id, (void**)data_ptrs);
        }

        template<typename T>
        void distribute(T** data_ptrs, uint32_t data_count)
        {
            if (id == -1) return;
            if (channel_count != data_count) api->log(api->ctx, (uint8_t*)"Distributor data count mismatch: expected %d, got %d", channel_count, data_count);
            else api->distribute(api->ctx, id, (void**)data_ptrs);
        }

        template<typename T>
        void distribute(const std::vector<T*>& data_ptrs) {distribute((T**)data_ptrs.data(), data_ptrs.size());}

        template<typename T>
        void distribute_data_vector(const std::vector<T>& data)
        {
            if (channel_count != data.size()) api->log(api->ctx, (uint8_t*)"Distributor data count mismatch: expected %d, got %d", channel_count, data.size());
            else
            {
                for (uint32_t i = 0; i < channel_count; i++) ptr_buffer[i] = (void*)&data[i];
                distribute(data.data(), data.size());
            }
        }

        template<typename... Args>
        void distribute(Args* ...args)
        {
            void* data[] = { (void*)args... };
            distribute(data, sizeof...(Args));
        }

        template<typename... Args>
        void enqueue(Args* ...ptrs)
        {
            if (queue_index + sizeof...(Args) > channel_count) return;

            uint32_t i = 0;
            ((ptr_buffer[queue_index+(i++)] = (void*)ptrs), ...);
            queue_index += sizeof...(Args);
        }

        void distribute_enqueued(bool clear = true)
        {
            distribute(ptr_buffer.data(), queue_index);
            if (clear) queue_index = 0;
        }

        void clear_queue() {queue_index = 0;}
        
    protected:
        Distributor(pond_api* _api, const ChannelsInfo& channels_info) : ptr_buffer(channels_info.infos.size())
        {
            api = _api;
            channel_count = channels_info.infos.size();
            id = api->create_distributor(api->ctx, (pond_dds_slot_info*)channels_info.infos.data(), channel_count);
        }

        int32_t id;
        pond_api* api;
        uint32_t channel_count;
        std::vector<void*> ptr_buffer;
        uint32_t queue_index = 0;
    };

    template<typename... Args>
    class DistributorTyped : public Distributor
    {
    friend class ModuleBase;
    public:
        DistributorTyped() = default;
        void distribute(Args* ...args)
        {
            void* data[] = { (void*)args... };
            Distributor::distribute(data, sizeof...(Args));
        }
    private:
        ChannelsInfo make_infos(const std::array<std::string, sizeof...(Args)>& channels)
        {
            ChannelsInfo info; auto it = channels.begin();
            (info.channel<Args>(*it++),...);
            return info;
        }

        DistributorTyped(pond_api* _api, const std::array<std::string, sizeof...(Args)>& channels) : Distributor(_api, make_infos(channels)) {}
    };

    class Receiver
    {
    friend class ModuleBase;
    public:
        Receiver() = default;
        void destroy()
        {
            if (id == -1) return;
            api->destroy_receiver(api->ctx, id);
        }
    private:
        Receiver(pond_api* _api, const ChannelsInfo& channels_info, std::shared_ptr<void>& callback, pfn_pond_receiver_callback callback_function)
        {
            api = _api;
            callback_ptr = callback;

            id = api->create_receiver(
                api->ctx, 
                (pond_dds_slot_info*)channels_info.infos.data(), 
                channels_info.infos.size(), 
                callback_function,
                callback_ptr.get()
            );
        }

        int32_t id;
        pond_api* api;
        std::shared_ptr<void> callback_ptr;
    };
}