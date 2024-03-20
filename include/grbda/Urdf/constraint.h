#ifndef URDF_INTERFACE_CONSTRANT_H
#define URDF_INTERFACE_CONSTRANT_H

#include "link.h"

namespace urdf
{

    class Constraint
    {
    public:
        Constraint() { this->clear(); };

        std::string name;
        enum
        {
            UNKNOWN,
            POSITION,
            ROLLING
        } type;

        std::string predecessor_link_name;
        std::string successor_link_name;

        /// subtrees rooted at the nearest common ancestor
        ///   the subtrees begin at (but do not include) the nearest common ancestor and end at
        ///   (and do include) the predecessor/successor links
        std::vector<std::shared_ptr<Link>> nca_to_predecessor_subtree, nca_to_successor_subtree;

        /// The following members are constraint specific. They are nullptrs by default, and are 
        ///     only set if the constraint is of the corresponding type.
        
        // Position constraint specific members
        //  The transform from the predecessor link frame to the constraint frame
        std::shared_ptr<Pose> predecessor_to_constraint_origin_transform;
        //  The transform from the successor link frame to the constraint frame
        std::shared_ptr<Pose> successor_to_constraint_origin_transform;

        // Rolling constraint specific members
        //  The ratio of the radius of the rolling constraint of the predecessor link to the radius 
        //  of the rolling constraint of the successor link
        std::shared_ptr<double> ratio;

        // TODO(@MatthewChignoli): add stuff for the differential constraint

        void clear()
        {
            this->name.clear();
            this->successor_link_name.clear();
            this->predecessor_link_name.clear();
            this->nca_to_predecessor_subtree.clear();
            this->nca_to_successor_subtree.clear();
            this->predecessor_to_constraint_origin_transform.reset();
            this->successor_to_constraint_origin_transform.reset();
            this->ratio.reset();
            this->type = UNKNOWN;
        };
    };

}

#endif
