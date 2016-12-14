/*
 * CUDAAgentStateList.cpp
 *
 *  Created on: 20 Feb 2014
 *      Author: paul
 */

#include <cuda_runtime.h>
#include <device_launch_parameters.h>

#include "CUDAAgentStateList.h"
#include "CUDAErrorChecking.h"



CUDAAgentStateList::CUDAAgentStateList(CUDAAgent& cuda_agent) : agent(cuda_agent)
{

    //allocate state lists
    allocateDeviceAgentList(&d_list);
    allocateDeviceAgentList(&d_swap_list);
    if (agent.getAgentDescription().requiresAgentCreation()) // Moz:  how about in 'CUDAAgentModel::simulate'?
        allocateDeviceAgentList(&d_new_list);
	else
	{
		//set new list hash map pointers to zero
		d_new_list.d_d_memory = 0;
		d_new_list.h_d_memory = 0;
	}

}

CUDAAgentStateList::~CUDAAgentStateList()
{
    //cleanup
    releaseDeviceAgentList(&d_list);
    releaseDeviceAgentList(&d_swap_list);
	if (agent.getAgentDescription().requiresAgentCreation())
        releaseDeviceAgentList(&d_new_list);
}

void CUDAAgentStateList::allocateDeviceAgentList(CUDAAgentMemoryHashMap* memory_map)
{
	//we use the agents memory map to iterate the agent variables and do allocation within our GPU hash map
    const MemoryMap &mem = agent.getAgentDescription().getMemoryMap();

    //allocate host vector (the map) to hold device pointers
	memory_map->h_d_memory = (void**)malloc(sizeof(void*)*agent.getHashListSize());
	//set all map values to zero
	memset(memory_map->h_d_memory, 0, sizeof(void*)*agent.getHashListSize());

    //for each variable allocate a device array and register in the hash map
	for (const MemoryMapPair mm : mem)
    {
	
		//get the hash index of the variable so we know what position to allocate in the map
		int hash_index = agent.getHashIndex(mm.first.c_str());

		//get the variable size from agent description
		size_t var_size = agent.getAgentDescription().getAgentVariableSize(mm.first);

		//do the device allocation at the correct index and store the pointer in the host hash map
		gpuErrchk(cudaMalloc((void**)&(memory_map->h_d_memory[hash_index]), var_size * agent.getMaximumListSize()));
    }

	//allocate device vector (the map) to hold device pointers (which have already been allocated)
	gpuErrchk(cudaMalloc((void**)&(memory_map->d_d_memory), sizeof(void*)*agent.getHashListSize()));

	//copy the host array of map pointers to the device array of map pointers
	gpuErrchk(cudaMemcpy(memory_map->d_d_memory, memory_map->h_d_memory, sizeof(void*)*agent.getHashListSize(), cudaMemcpyHostToDevice));



}

void CUDAAgentStateList::releaseDeviceAgentList(CUDAAgentMemoryHashMap* memory_map)
{
	//we use the agents memory map to iterate the agent variables and do deallocation within our GPU hash map
    const MemoryMap &mem = agent.getAgentDescription().getMemoryMap();
    
	//for each device pointer in the map we need to free these
	for (const MemoryMapPair mm : mem)
    {
		//get the hash index of the variable so we know what position to allocate
		int hash_index = agent.getHashIndex(mm.first.c_str());

		//free the memory on the device
		gpuErrchk(cudaFree(memory_map->h_d_memory[hash_index]));
    }

	//free the device memory map
	gpuErrchk(cudaFree(memory_map->d_d_memory));

	//free the host memory map
	free(memory_map->h_d_memory);
}


void CUDAAgentStateList::setAgentData(const AgentStateMemory &state_memory)
{

    //check that we are refering to the same agent description
    if (!state_memory.isSameDescription(agent.getAgentDescription()))
    {
        //throw std::runtime_error("CUDA Agent uses different agent description.");
        throw InvalidCudaAgentDesc();
    }

    //copy raw agent data to device pointers
    const MemoryMap &mem = agent.getAgentDescription().getMemoryMap();

    int i=0;
    for (MemoryMap::const_iterator it = mem.begin(); it != mem.end(); it++)
    {

        //gpuErrchk( cudaMemcpy( d_Circles_default, h_Circles_default, xmachine_Circle_SoA_size, cudaMemcpyHostToDevice));

        //gpuErrchk( cudaMalloc( (void**) &((*agent_list)->h_d_memory[i]), agent_description.getAgentVariableSize(it->first) * max_list_size));
        i++;
    }


}

