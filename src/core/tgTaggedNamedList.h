/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file tgTaggedNamedList.h
 * @brief Contains the definition of class tgTaggedNamedList
 * @author Ryan Adams
 * $Id$
 */

#ifndef TG_TAGGED_NAMED_LIST_H
#define TG_TAGGED_NAMED_LIST_H

#include <iostream> // Testing only
#include <algorithm>
#include <vector>
#include <map>
#include <stdexcept>
#include "tgTaggable.h"
class tgTaggable;

template <class T>
class tgTaggedNamedList
{
public:
    
    tgTaggedNamedList() {
        // Postcondition
        assert(m_elements.empty());
    };

    /**
     * Create a set of elements given a vector of T.
     * @param[in] nodes a vector of T; the elements must be unique
     * @author Lee Brownston
     * @date Wed 26 Feb 2014
     */
    tgTaggedNamedList(std::vector<T>& elements) : m_elements(elements) {
        // All elements must be unique
        assertUniqueElements("All elements must be unique.");
        
    };
        
    virtual ~tgTaggedNamedList() {};
    
    /**
     * Return a vector of pointers to Ts that have all of
     * the specified tags.
     */
    std::vector<T*> find(std::string tags) 
    {
        std::vector<T*> result;
        for(int i = 0; i < m_elements.size(); i++) {
            if(_taggable(&m_elements[i])->hasAllTags(tags)) {
                result.push_back(&(m_elements[i]));
            }
        }
        return result;
    }
    
    int size() const
    {
        return m_elements.size();
    }
    
    std::vector<T*> findAll()
    {
        std::vector<T*> result;
        for(int i = 0; i < m_elements.size(); i++) {
            result.push_back(&(m_elements[i]));
        }
        return result;
    }
    
    std::vector<T*> findUntagged()
    {
        std::vector<T*> result;
        for(int i = 0; i < m_elements.size(); i++) {
            tgTaggable* t = _taggable(&m_elements[i]);
            if(t->hasNoTags()) {
                result.push_back(&(m_elements[i]));
            }
        }
        return result;
    }

    static bool contains(std::vector<T*> haystack, const T* needle)
    {
        return std::find(haystack.begin(), haystack.end(), needle) != haystack.end();
    }

    bool contains(const T& needle) const
    {
        //return std::find(m_elements.begin(), m_elements.end(), needle) != m_elements.end(); // generates errors??
        for(typename std::vector<T>::const_iterator it = m_elements.begin(); it != m_elements.end(); it++) {
            if(&(*it) == &needle)
                return true;
        }
        return false;
    }
        
    /**
     * Return a non-const reference to the element that is indexed by the
     * int key. It must be in m_elements.
     * @param[in] key the key of the element to retrieve
     * @return a const reference to the element that is indexed by idx
     */
    T& operator[](int key) { 
        assertKeyExists(key);
        return m_elements[key]; 
    }
    
    const T& operator[](int key) const { 
        assertKeyExists(key);
        return m_elements[key]; 
    }

    /**
     * Return a non-const reference to the element named by name. It must be 
     * in m_names. Note that there is no const access by name as the names are
     * a map (since the [] operator on a map will automatically add an element,
     * a const accessor would cause a compiler error). 
     */
    T& operator[](const std::string& name) 
    { 
        assertNameExists(name);
        int idx = m_names[name];
        return this[idx];
    }
    
    const T& operator[](const std::string& name) const
    {
        assertNameExists(name);
        // Note: we know that the name is present since we checked with 
        // assertNameExists(), which thrown an exception if name is not found.
        return this[m_names.find(name)->second];
    }
    
    /**
     * Remove the elements contained in 'other' from this object
     */
    /* 
    // @note: Need to refactor removeElements to take into account named 
    // elements. Commented out until that can happen. 
    
    T& operator-=(const T& other) {
        this->removeElements(other.getElements());
        return *this;
    }

    T& operator-=(const std::vector<T*> other) {
        this->removeElements(other);
        return *this;
    }
    */

    T& operator+=(const T& other) {
        this->addElement(other);
        return *this;
    }

    T& operator+=(const std::vector<T*> other) {
        this->addElements(other);
        return *this;
    }


protected:
    
    // @todo: think about uniqueness -- if not unique, throw an error? return -1? 
    int addElement(T element) 
    {
        // @todo: make sure the element is unique
        assert(!elementExists(element));  // segfault?
        assertUnique(element);
        
        // Note: not thread safe
        int idx = m_elements.size();
        m_elements.push_back(element);
        return idx;  // This is the index that was created.
    }

    int addNamedElement(const std::string& name, T element)
    {
        assertNameUnused(name);
        int idx = addElement(element);
        m_names[name] = idx;
        return idx;
    }

    void addElements(std::vector<T*> elements) 
    {
        for(int i = 0; i < elements.size(); i++) {
            this->addElement(elements[i]);
        }
    }

    void setElement(int key, T element) {
        assert((0 <= key) && (key <= m_elements.size()));
        m_elements[key] = element;
    }
        
    std::vector<T>& getElements() 
    {
        return m_elements;
    };

    const std::vector<T>& getElements() const
    {
        return m_elements;
    };

    /* 
    // @note: disabling removal of elements until it can be refactored to 
    // account for named elements. 
    void removeElement(const T& element) {
        m_elements.erase(std::remove(m_elements.begin(), m_elements.end(), element), m_elements.end());
    }

    void removeElement(const T* element) {
        m_elements.erase(std::remove(m_elements.begin(), m_elements.end(), *element), m_elements.end());
    }

    void removeElements(const std::vector<T>& elements) {
        for(int i = 0; i < elements.size(); i++) {
            removeElement(elements[i]);
        }
    }

    void removeElements(const std::vector<T*>& elements) {
        for(int i = 0; i < elements.size(); i++) {
            removeElement(elements[i]);
        }
    }
    */
    
    // To make subclassing operators easier...
    T& getElement(int key) 
    {
        return m_elements[key];
    }

    // To make subclassing operators easier...
    const T& getElement(int key) const
    {
        return m_elements[key];
    }


    T& getElement(std::string name)
    {
        assertNameExists(name);
        return this[m_names[name]];
    }

    /**
     * Set the index to which a name points. 
     */
    int setNameIndex(const std::string& name, int idx)
    {
        m_names[name] = idx;
        return idx;  // @note: return the idx to help with fluent interfaces
    }

    /**
     * Is the index within range.
     * @param[in] key an int
     * @retval true if key is within range
     * @retval false if key is not within range
     */
    bool keyExists(int key) const
    {
        return (0 <= key) && (key < m_elements.size());
    }        
    
    bool nameExists(std::string name) const
    {
        return m_names.count(name) != 0;
    }
    
    // @todo: FIX THIS -- segfaults, etc. -- what's going on? 
    bool elementExists(const T& element) const
    {        
        //return std::find(m_elements.begin(), m_elements.end(), element) != m_elements.end(); // segfault?
        for(int i = 0; i < m_elements.size(); i++) {
            // This is a little strange to me, but at least it doesn't cause a segfault...
            const T* elem =& m_elements[i];
            if (elem == &element) {
                return true;
            }
        }
        return false;
    }
    
    void assertKeyExists(int key, std::string message = "Element at index does not exist") const
    {
        if(!keyExists(key)) {
            std::stringstream ss; 
            ss << key;
            throw std::out_of_range(message + " (index "+ ss.str() + ").");
        }        
    }
    
    void assertNameExists(std::string name, std::string message = "Name pointer does not exist") const
    {
        if(!nameExists(name)) {
            std::stringstream ss; 
            ss << name;
            throw std::out_of_range(message + " (name '"+ ss.str() + "').");
        }        
    }
    
    void assertNameUnused(std::string name, std::string message = "Name already used") const
    {
        if(nameExists(name)) {
            std::stringstream ss; 
            ss << name;
            throw std::logic_error(message + " (name '"+ ss.str() + "').");
        }        
    }
    
    void assertUnique(T& element, std::string message = "Taggable elements must be unique.") {
        if(elementExists(element)) {
            throw std::logic_error(message);
        }
    }
    
    void assertUniqueElements(std::string message = "Taggable elements must be unique.") const
    {
        /* Note: this throws a "Most vexing parse" error (http://en.wikipedia.org/wiki/Most_vexing_parse)
        // Note: This would probably work if we implemented operator< on tgPair...
        */
        
        if(! std::set<T>(m_elements.begin(), m_elements.end()).size() ==
             m_elements.size()) {
                 throw std::logic_error(message);
             }

    }
    
    // Cast T to taggable (after all, T must be a tgTaggable in the first place, but )
    // there doesn't seem to be a way to enforce that with c++ templates...
    tgTaggable* _taggable(T* obj) {
        return static_cast<tgTaggable*>(obj);
    }
    
private:
    std::vector<T> m_elements;
    std::map<std::string, int> m_names;
};


#endif
