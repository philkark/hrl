template<class T>
  class Toggle
  {
  public:
    T* current;
    T* next;

    Toggle()
    {
      current = &obj0;
      next = &obj1;
      currentObject = 0;
    }

    void toggle()
    {
      if (currentObject == 0)
      {
        current = &obj1;
        next = &obj0;
        currentObject = 1;
      }
      else
      {
        current = &obj0;
        next = &obj1;
        currentObject = 0;
      }
    }

  private:
    T obj0;
    T obj1;

    int currentObject;
  };
