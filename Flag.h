#ifndef _FLAG_
#define _FLAG_

class Flag   //
{
    public:
        enum Method {       //実施内容
            RUN_UNDEF,      //未定義
            RUN_SPEED,      //タイムアタック
            RUN_LINE,       //ライントレース
            RUN_ONLINE,     //ライントレース(黒線寄り)
            RUN_STRAIGHT,   //直進
            RUN_TURN,       //旋回
            RUN_THROW,      //スローイン
            RUN_ARM,        //アーム操作
            RUN_COLOR,      //ブロックのカラーチェック
            RUN_SAVE,       //ブロック色の保存
            RUN_RECORD,     //レコードカウント
            RUN_EDGE,       //エッジ制御
            RUN_END         //終了
        };

        enum End {          //終了条件
            END_UDF,        //未定義
            END_LEN,        //距離
            END_ANG,        //角度
            END_ANG2,       //
            END_ARM,        //アーム角度
            END_COL,        //色（赤、青、緑、黄）
            END_CNT,        //時間超過
            END_BLK,        //黒
            END_ALL         //無条件終了
        };

    private:

};

#endif
