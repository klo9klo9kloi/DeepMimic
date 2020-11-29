#pragma once

#include "scenes/RLSceneSimChar.h"
#include "anim/KinCharacter.h"

class cSceneImitate : virtual public cRLSceneSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSceneImitate();
	virtual ~cSceneImitate();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();

	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;
	virtual void EnableRandRotReset(bool enable);
	virtual bool EnabledRandRotReset() const;

	virtual double CalcReward(int agent_id) const;;
	virtual eTerminate CheckTerminate(int agent_id) const;

	virtual std::string GetName() const;

	virtual void RecordState(int agent_id, Eigen::VectorXd out_state) const;
	virtual int GetStateSize(int agent_id) const;
	virtual void BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;

protected:

	std::string mMotionFile;
	std::shared_ptr<cKinCharacter> mKinChar;

	Eigen::VectorXd mJointWeights;
	bool mEnableRandRotReset;
	bool mSyncCharRootPos;
	bool mSyncCharRootRot;
	bool mEnableRootRotFail;
	bool mAugment; //@klo9klo9kloi
	int mK; //@klo9klo9kloi
	double mHoldEndFrame;
	double mBaseMotionDuration; // @klo9klo9kloi

	virtual bool BuildCharacters();

	virtual void CalcJointWeights(const std::shared_ptr<cSimCharacter>& character, Eigen::VectorXd& out_weights) const;
	virtual bool BuildController(const cCtrlBuilder::tCtrlParams& ctrl_params, std::shared_ptr<cCharController>& out_ctrl);
	virtual void BuildKinChar();
	virtual bool BuildKinCharacter(int id, std::shared_ptr<cKinCharacter>& out_char) const;
	virtual void UpdateCharacters(double timestep);
	virtual void UpdateKinChar(double timestep);

	virtual void ResetCharacters();
	virtual void ResetKinChar();
	virtual void SyncCharacters();
	virtual bool EnableSyncChar() const;
	virtual void InitCharacterPosFixed(const std::shared_ptr<cSimCharacter>& out_char);

	virtual void InitJointWeights();
	virtual void ResolveCharGroundIntersect();
	virtual void ResolveCharGroundIntersect(const std::shared_ptr<cSimCharacter>& out_char) const;
	virtual void SyncKinCharRoot();
	virtual void SyncKinCharNewCycle(const cSimCharacter& sim_char, cKinCharacter& out_kin_char) const;

	virtual double GetKinTime() const;
	virtual bool CheckKinNewCycle(double timestep) const;
	virtual bool HasFallen(const cSimCharacter& sim_char) const;
	virtual bool CheckRootRotFail(const cSimCharacter& sim_char) const;
	virtual bool CheckRootRotFail(const cSimCharacter& sim_char, const cKinCharacter& kin_char) const;
	
	virtual double CalcRandKinResetTime();
	virtual void SetRandKinMotionTime(); // @klo9klo9kloi
    virtual double CalcRewardImitate(const cSimCharacter& sim_char, const cKinCharacter& ref_char) const;
	virtual void SetRandKinMotionTime();
    virtual double CalcRewardImitate(const cSimCharacter& sim_char, const cKinCharacter& ref_char) const;
    virtual std::vector<double> CalcAugmentedStates(const cKinCharacter& ref_char,int k) const;
};
