#ifndef ARIS_CONTROL_MOTION_H
#define ARIS_CONTROL_MOTION_H

#include <functional>
#include <thread>
#include <atomic>
#include <array>

#include <aris_control_ethercat.h>


namespace aris
{
	namespace control
	{	
		class EthercatMotion :public EthercatSlave
		{
		public:
			enum Cmd
			{
				IDLE = 0,
				ENABLE,
				DISABLE,
				HOME,
				RUN
			};
			enum Mode
			{
				POSITION = 0x0008,
				VELOCITY = 0x0009,
				CURRENT = 0x0010,
			};
			struct RawData
			{
				std::int32_t target_pos{ 0 }, feedback_pos{ 0 };
				std::int32_t target_vel{ 0 }, feedback_vel{ 0 };
				std::int16_t target_cur{ 0 }, feedback_cur{ 0 };
				std::uint8_t cmd{ IDLE };
				std::uint8_t mode{ POSITION };
				mutable std::int16_t ret{ 0 };
			};

			virtual ~EthercatMotion();
			EthercatMotion(const aris::core::XmlElement &xml_ele, const aris::core::XmlElement &type_xml_ele);
			auto hasFault()->bool;
			auto readFeedback(RawData &data)->void;
			auto writeCommand(const RawData &data)->void;
			auto absID()->std::int32_t;
			auto phyID()->std::int32_t;
			auto maxPosCount()->std::int32_t;
			auto minPosCount()->std::int32_t;
			auto maxVelCount()->std::int32_t;
			auto pos2countRatio()->std::int32_t;
			auto setPosOffset(std::int32_t offset)->void;
			auto posOffset()const->std::int32_t;

		private:
			class Imp;
			std::unique_ptr<Imp> imp_;

			friend class EthercatController;
		};
		class EthercatForceSensor final:public EthercatSlave
		{
		public:
			struct Data
			{
				union
				{
					struct { double Fx, Fy, Fz, Mx, My, Mz; };
					double fce[6];
				};
			};

			EthercatForceSensor(const aris::core::XmlElement &xml_ele): EthercatSlave(xml_ele){};
			auto readData(Data &data)->void;

		protected:
			virtual auto init()->void override
			{
				this->readSdo(0, force_ratio_);
				this->readSdo(1, torque_ratio_);
			};
			std::int32_t force_ratio_, torque_ratio_;
		};

		class EthercatForceSensorRuiCongCombo final:public EthercatSlave
		{
		public:
			struct RuiCongComboData
			{
				union Data
				{
					struct { double Fx, Fy, Fz, Mx, My, Mz; };
					double fce[6];
				};
				std::array<Data,6> force;
				std::array<bool, 6> isZeroingRequested = { {false,false,false,false,false,false} };
			};
			EthercatForceSensorRuiCongCombo(const aris::core::XmlElement &xml_ele) : EthercatSlave(xml_ele) {};

			auto readData(RuiCongComboData &data)->void;
			
			// set ratio
			auto setRatio(double f_ratio=0.001,double t_ratio=0.001)->void;// kg, kgm

			// clear
			auto requireZeroing(int sensor_id) -> void;

		protected:
			virtual auto init()->void override
			{
				this->setRatio();
				//clear values
				for (int i = 0; i < 6; i++)
				{
					base_data_.force.at(i).Fx = 0;
					base_data_.force.at(i).Fy = 0;
					base_data_.force.at(i).Fz = 0;
					base_data_.force.at(i).Mx = 0;
					base_data_.force.at(i).My = 0;
					base_data_.force.at(i).Mz = 0;

					sum_data_.force.at(i).Fx = 0;
					sum_data_.force.at(i).Fy = 0;
					sum_data_.force.at(i).Fz = 0;
					sum_data_.force.at(i).Mx = 0;
					sum_data_.force.at(i).My = 0;
					sum_data_.force.at(i).Mz = 0;
				}

			};

			/*static const int ZEROING_COUNT = 500;*/
			static const int ZEROING_COUNT = 1;

			std::array<int, 6> zeroing_count_left = { { -1,-1,-1,-1,-1,-1 } };
			// for zeroing
			RuiCongComboData base_data_;
			RuiCongComboData sum_data_;
			RuiCongComboData raw_data_;

			double force_ratio_=0.001, torque_ratio_=0.001;
		};

		class EthercatController :public EthercatMaster
		{
		public:
			struct Data
			{
				const std::vector<EthercatMotion::RawData> *last_motion_raw_data;
				std::vector<EthercatMotion::RawData> *motion_raw_data;
				std::vector<EthercatForceSensor::Data> *force_sensor_data;
				std::vector<EthercatForceSensorRuiCongCombo::RuiCongComboData> *ruicongcombo_data;
				const aris::core::MsgRT *msg_recv;
				aris::core::MsgRT *msg_send;
			};

			virtual ~EthercatController();
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void override;
			virtual auto start()->void;
			virtual auto stop()->void;
			auto setControlStrategy(std::function<int(Data&)>)->void;
			auto motionNum()->std::size_t;
			auto motionAtAbs(int i)->EthercatMotion &;
			auto motionAtPhy(int i)->EthercatMotion &;
			auto forceSensorNum()->std::size_t;
			auto forceSensorAt(int i)->EthercatForceSensor &;
			auto ruicongComboNum()->std::size_t;
			auto ruicongComboAt(int i)->EthercatForceSensorRuiCongCombo &;
			auto msgPipe()->Pipe<aris::core::Msg>&;

		protected:
			EthercatController();
			virtual auto controlStrategy()->void override final;

		private:
			struct Imp;
			std::unique_ptr<Imp> imp_;

			friend class EthercatMaster;
		};
	}
}

#endif
