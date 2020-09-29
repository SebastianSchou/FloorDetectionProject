class HelperFunction{
public:
	HelperFunction();
	~HelperFunction();

	float msUntilNow(const std::chrono::steady_clock::time_point& start);
};