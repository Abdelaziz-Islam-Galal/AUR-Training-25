from random import randint

class WrongManagerError(Exception):
  pass

class Employee:
    def __init__(self, name, family, manager=None):
      self._name = name
      self._id = randint(1000,9999)
      self._family = family.copy()
      self._manager = manager
      self.salary = 2500
    
    @property
    def id(self) -> int:
      return self._id

    @property
    def family(self) -> dict:
      return self._family.copy()
    
    def apply_raise(self, managed_employee: 'Employee', raise_percent: int):
      if not isinstance(managed_employee, Employee):
        raise TypeError("type must be Employee")
      elif not managed_employee._manager == self:
        raise WrongManagerError(f"{self} is not the manager for {managed_employee}")
      else:
        raise_percent += 100
        percent = raise_percent / 100
        managed_employee.salary *= percent
        print(f"new salary is ${managed_employee.salary}")
        return True

      # you must handle error where managed_employee._manager isn't self
      # choose to return success status or raise exceptions


    
##### Test code: #####
if __name__ == '__main__':
  boss = Employee('Jane Redmond', {}) # name: Jane Redmond, family: {}, manager: none
  name = 'John Smith'
  family = {
    'Son': {
      'Insured': True,
      'Age': 16
    },
    'Wife': {
      'Insured': False,
      'Age': 32
    }
  }
  my_employee = Employee(name, family, boss)
  not_boss = Employee('Adam Cater', {})
  # do not change:
  print(id(my_employee.family))
  print(id(my_employee._family)) # should be different
  boss.apply_raise(my_employee, 25)
  print(not_boss.apply_raise(my_employee, 25))