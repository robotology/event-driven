#ifndef EVENTATOM_H
#define EVENTATOM_H

namespace emorph
{
namespace evolume
{

class eventAtom
{
public:
    eventAtom(){};
    eventAtom(unsigned int, unsigned int, short, unsigned int);
    ~eventAtom();

    inline void set_follower(eventAtom* _follower){follower=_follower;};

    inline unsigned int get_x(){return addrx;};
    inline unsigned int get_y(){return addry;};
    inline int get_pol(){return pol;};
    inline unsigned int get_ts(){return ts;};
    inline eventAtom* get_follower(){return follower;};
private:
    int pol;
    unsigned int addrx;
    unsigned int addry;
    unsigned int ts;

    eventAtom* follower;
};

}
}
#endif //EVENTATOM_H
