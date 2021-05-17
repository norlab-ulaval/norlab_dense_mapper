#include "RAMCellManager.h"

std::vector<std::string> norlab_dense_mapper::RAMCellManager::getAllCellIds() const
{
	std::vector<std::string> cellIds;
	for(const auto& cell : cells)
	{
		cellIds.push_back(cell.first);
	}
	return cellIds;
}

void norlab_dense_mapper::RAMCellManager::saveCell(const std::string& cellId, const PM::DataPoints& cell)
{
	cells[cellId] = cell;
}

norlab_dense_mapper::CellManager::PM::DataPoints norlab_dense_mapper::RAMCellManager::retrieveCell(const std::string& cellId) const
{
	PM::DataPoints cell;
	if(cells.find(cellId) != cells.end())
	{
		cell = cells.at(cellId);
	}
	return cell;
}

void norlab_dense_mapper::RAMCellManager::clearAllCells()
{
	cells.clear();
}
