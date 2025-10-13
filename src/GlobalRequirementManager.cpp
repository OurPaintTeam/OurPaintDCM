#include "GlobalRequirementManager.h"

OurPaintDCM::GlobalRequirementManager::GlobalRequirementManager()
    : _reqs(),
      _reqsData(),
      _graph(),
      _mode(Utils::SolveMode::GLOBAL) {

}

OurPaintDCM::GlobalRequirementManager::~GlobalRequirementManager() {
    for (auto& [id, req] : _reqs) {
        delete req;
    }
}

void OurPaintDCM::GlobalRequirementManager::addRequirement(Utils::RequirementData req) {
    if (!req.requirement) {
        throw std::invalid_argument("Requirement is nullptr.");
    }

    Utils::ID id = _idGenerator.nextID();
    req.id = id;
    // Make sure we don't have another owner to this req
    _reqs[id] = req.requirement;
    req.requirement = nullptr;
    // Add to graph of dependencies
    for (auto& data : req.objects) {
        Utils::ID elem_id = data.id;
        _graph.addVertex(data.id);
        for (auto& subobj: data.subObjects) {
            _graph.addVertex(subobj);
            _graph.addEdge(elem_id, subobj, Utils::ID(-10));
        }
    }
    if (req.objects.size() == 1) {
        _graph.addEdge(req.objects[0].id, req.objects[0].id, id);
    }
    if (req.objects.size() > 1) {
        for (size_t i = 1; i < req.objects.size(); ++i) {
            _graph.addEdge(req.objects[i-1].id, req.objects[i].id, id);
        }
    }
}

void OurPaintDCM::GlobalRequirementManager::removeRequirement(Utils::ID id) {
    if (!_reqs.contains(id) || id < 0) {
        throw std::invalid_argument("Requirement with id " + std::to_string(id.id) + " does not exist.");
    }
    for (auto it = _reqsData.begin(); it != _reqsData.end(); ++it) {
        const Utils::RequirementData& req = *it;
        if (req.id == id) {
            if (req.objects.size() == 1) {
                _graph.removeEdge(req.objects[0].id, req.objects[1].id);
            }
            if (req.objects.size() > 1) {
                for (size_t i = 1; i < req.objects.size(); ++i) {
                    _graph.removeEdge(req.objects[i-1].id, req.objects[i].id);
                }
            }
            _reqs.erase(id);
            _reqsData.erase(it);
            _components.deleteElement(id);
            break;
        }
    }
}

void OurPaintDCM::GlobalRequirementManager::removeAllRequirements() {
    _graph = Graph<Utils::ID, Utils::ID>();
    _reqs.clear();
    _reqsData.clear();
    _components.clear();
}
