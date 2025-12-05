from .test_llm import llm_loop

test_texts = \
    '''{"pre_confirmed": false, "text": "请帮我添加一个明天下午三点去健身房的日程。"}
    {"pre_confirmed": false, "text": "我下周二好像要去看牙医。"}
    {"pre_confirmed”: false, “text”: “预约下个月15号上午9:30与张经理的电话会议。”}
    {"pre_confirmed": false, "text": “原定下周一上午的团队周会，需要改到下午。”}
    {"pre_confirmed": false, "text": “把周六‘超市采购’的日程描述改成‘家庭采购日’。”}
    {"pre_confirmed”: false, “text”: “请取消12月25号全天的所有日程。”}
    {"pre_confirmed”: false, “text”: “帮我查一下下周三下午四点的评审会。”}
    {"pre_confirmed": false, "text": “我下周哪天跟王总吃饭来着？”}
    {"pre_confirmed”: false, “text”: “今天的天气真不错。”}
    {"pre_confirmed”: false, “text”: “明天要交报告。哦对了，周日晚上记得提醒我给妈妈打电话。冰箱里没牛奶了。”}
    {"pre_confirmed”: true, “text”: “-我下周五晚上有聚会。-什么聚会？-哦不对，是公司年会，时间改成下周五下午六点了。”}
    {"pre_confirmed”: false, “text”: “帮我把‘项目复盘’这个日程的名称改成‘季度项目复盘’。”}
    {"pre_confirmed”: false, “text”: “对了，重要的事，请安排本周日上午十点健身，别忘了。”}
    {"pre_confirmed”: false, “text”: “大后天傍晚记得收快递。”}
    {"pre_confirmed”: false, “text”: “我之前说的那个大概下周的聚餐，取消掉吧。”}
    {"pre_confirmed”: true, “text”: “-你有一个明天下午三点的会议。-好的。”}
    {"pre_confirmed”: false, “text”: “我明天有什么安排？如果没事，就帮我约晚上七点理发。”}
    {"pre_confirmed”: false, “text”: “帮我设置一个中午十二点半和莉莉吃午饭的日程。”}
    {"pre_confirmed”: false, “text”: “将‘编写月报’从周五改到下周一下午。”}
    {"pre_confirmed”: false, “text”: “下个月我得出差去上海一趟，估计要去三天。”}'''

def test_extract_info():
    llm_loop("llm/extract_info", test_texts)