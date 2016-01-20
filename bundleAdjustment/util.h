
namespace helper
{
    inline int countBits(UINT64 v)
    {
        return (int)__popcnt64(v);
    }
}
