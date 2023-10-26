#ifndef _BUSH_H
#define _BUSH_H

//--Bush Interface---------------------------------------------------
class Bush
{        
    public:

        // Constructs a bush with a hazard inidicator given by hazard
        Bush(int iD, bool hazard, bool fire);

        // Deconstructs a bush
        ~Bush();

        // Returns the ID number of the bush
        int ID();

        // Outputs whether the bush is hazardous 
        // and returns the hazardous indicator as a boolean
        bool IsHazard();

        // Outputs whether the bush is on fire
        // and returns the fire indicator as a boolean
        bool OnFire();

        // Performs controlled burning on the bush
        void ControlledBurning();

        // Eliminates the bush fire
        void EliminateFire();

    private:

        // The ID number of the bush
        int iD;

        // Inidicates whether the bush is a bushfire hazard
        bool hazard;

        // Inidicates whether the bush is on fire
        bool fire;

};

#endif
