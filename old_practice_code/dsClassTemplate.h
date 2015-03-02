#ifndef dsClassTemplate_H
#define dsClassTemplate_H

template<typename Object>
class MemoryCell{
	public:
		explicit MemoryCell(const Object & initialValue = Object()):m_storedValue(initialValue) {}
		const Object & read() const{
			return m_storedValue;
		}

		void write(const Object & x){
			m_storedValue = x;
		}

	private:
		Object m_storedValue;

};

#endif
