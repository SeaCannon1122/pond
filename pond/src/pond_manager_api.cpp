#include "pond_manager.hpp"

void PondManager::api_module_shutdown(pond_internal::Module* module)
{

}

void PondManager::api_module_log(pond_internal::Module* module, uint8_t* format, va_list args)
{

}

void PondManager::api_module_set_user_ptr(pond_internal::Module* module, void* ptr)
{

}

void* PondManager::api_module_get_user_ptr(pond_internal::Module* module)
{

}

int32_t PondManager::api_create_distributor(pond_internal::Module* module, uint8_t* topic)
{

}

void PondManager::api_destroy_distributor(pond_internal::Module* module, uint32_t distributor)
{

}

void PondManager::api_distribute(pond_internal::Module* module, uint32_t distributor, void* data)
{

}

int32_t PondManager::api_create_receiver(pond_internal::Module* module, uint8_t* topic, pfn_pond_receiver_callback callback, void* callback_pointer)
{

}

void PondManager::api_destroy_receiver(pond_internal::Module* module, uint32_t receiver)
{

}
