#ifndef WWU_LDPLAB_UID_HPP
#define WWU_LDPLAB_UID_HPP

#include <atomic>
#include <stddef.h>

namespace ldplab
{
    /** 
     * @brief A class that represents a unique ID right after construction.
     * @details The unique ID is type specific, meaning for every type the
     *          unique IDs are counted seperately. The first UID to be 
     *          created is always 0. UIDs are basically used to differentiate
     *          between multiple instances of the same class.
     * @warning Do not confuse this UID class with standardized UUIDs.
     */
    template<class SUPERSET_TYPE>
    class UID
    {
    public:
        /** @brief Immeadiatly constructs a UID. */
        UID();
        /** @brief Implicit conversion of instance to UID integer. */
        operator size_t() const;
    private:
        size_t m_uid;
    };

    template<class SUPERSET_TYPE>
    inline UID<SUPERSET_TYPE>::UID()
    {
        static std::atomic_size_t uid_counter{ 0 };
        m_uid = uid_counter.fetch_add(1);
    }

    template<class SUPERSET_TYPE>
    inline UID<SUPERSET_TYPE>::operator size_t() const
    {
        return m_uid;
    }
}

#endif